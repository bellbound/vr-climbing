#include "ClimbManager.h"
#include "ClimbSurfaceDetector.h"
#include "BallisticController.h"
#include "ClimbExitCorrector.h"
#include "CriticalStrikeManager.h"
#include "StaminaDrainManager.h"
#include "ClimbingDamageManager.h"
#include "HiggsCompatManager.h"
#include "EquipmentManager.h"
#include "MenuChecker.h"
#include "AudioManager.h"
#include "Config.h"
#include <algorithm>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

#include "util/VRNodes.h"
#include "util/Raycast.h"
#include <spdlog/spdlog.h>
#include <cmath>
#include <cstring>

// Layer filter for collision detection - returns true for solid world geometry
// Used to prevent phasing through walls while climbing
static bool IsSolidWorldLayer(RE::COL_LAYER layer)
{
    switch (layer) {
        case RE::COL_LAYER::kStatic:
        case RE::COL_LAYER::kAnimStatic:  // Moving platforms, gates, drawbridges
        case RE::COL_LAYER::kTerrain:
        case RE::COL_LAYER::kGround:
        case RE::COL_LAYER::kTrees:
        case RE::COL_LAYER::kProps:       // Larger props (furniture, etc.)
            return true;
        default:
            return false;
    }
}

// Fire a short haptic pulse when a hand successfully latches onto a surface.
// Uses SkyrimVR's internal BSVRInterface::TriggerHapticPulse via the g_openVR pointer,
// avoiding any dependency on linking openvr_api.lib.
// Reference implementation pattern: ImmersiveVRActions.cpp (LP_CallTriggerHapticPulse).

// SkyrimVR.exe global: BSOpenVR* g_openVR
// Offset sourced from SKSEVR GameVR.cpp
static REL::Relocation<void**> g_openVR{ REL::Offset(0x02FEB9B0) };

// Cached pointer - looked up once, reused thereafter
static void* s_cachedOpenVR = nullptr;
static bool s_openVRLookupDone = false;

static bool IsReadableAddress(const void* a_ptr, std::size_t a_bytes = sizeof(void*))
{
#if defined(_WIN32)
    if (!a_ptr) {
        return false;
    }

    MEMORY_BASIC_INFORMATION mbi{};
    if (::VirtualQuery(a_ptr, &mbi, sizeof(mbi)) == 0) {
        return false;
    }
    if (mbi.State != MEM_COMMIT) {
        return false;
    }

    const DWORD protect = mbi.Protect & 0xFF;
    switch (protect) {
        case PAGE_READONLY:
        case PAGE_READWRITE:
        case PAGE_WRITECOPY:
        case PAGE_EXECUTE_READ:
        case PAGE_EXECUTE_READWRITE:
        case PAGE_EXECUTE_WRITECOPY:
            break;
        default:
            return false;
    }

    if ((mbi.Protect & PAGE_GUARD) != 0) {
        return false;
    }

    const auto start = reinterpret_cast<std::uintptr_t>(a_ptr);
    const auto end = start + a_bytes;
    const auto regionStart = reinterpret_cast<std::uintptr_t>(mbi.BaseAddress);
    const auto regionEnd = regionStart + mbi.RegionSize;
    return end <= regionEnd;
#else
    (void)a_ptr;
    (void)a_bytes;
    return false;
#endif
}

static void* GetOpenVRInterface()
{
    // Return cached pointer if already looked up
    if (s_openVRLookupDone) {
        return s_cachedOpenVR;
    }
    s_openVRLookupDone = true;

#if defined(_WIN32)
    void** ppOpenVR = g_openVR.get();
    if (!IsReadableAddress(ppOpenVR)) {
        spdlog::warn("Haptics: g_openVR address not readable");
        return nullptr;
    }

    void* openVR = *ppOpenVR;
    if (!IsReadableAddress(openVR)) {
        spdlog::warn("Haptics: BSOpenVR instance not readable");
        return nullptr;
    }

    s_cachedOpenVR = openVR;
    return openVR;
#else
    return nullptr;
#endif
}

static void CallTriggerHapticPulse(void* a_openVR, int a_hand, float a_durationUnits)
{
    if (!a_openVR) {
        return;
    }

    // Get vtable pointer and validate it's readable
    void** vtblPtr = *reinterpret_cast<void***>(a_openVR);
    if (!IsReadableAddress(vtblPtr, sizeof(void*) * 15)) {
        spdlog::warn("Haptics: vtable not readable");
        return;
    }

    // BSVRInterface vtbl index for TriggerHapticPulse is 14 (see SKSEVR GameVR.h).
    constexpr std::size_t kVtblIndex_TriggerHapticPulse = 14;
    using Fn = void (*)(void*, int, float);
    auto fn = reinterpret_cast<Fn>(vtblPtr[kVtblIndex_TriggerHapticPulse]);
    if (!fn) {
        return;
    }

    fn(a_openVR, a_hand, a_durationUnits);
}

static void TriggerLatchHaptic(bool isLeft)
{
    void* openVR = GetOpenVRInterface();
    if (!openVR) {
        return;
    }

    // Hand enum in SKSEVR: Left=0, Right=1
    constexpr int kLeftHand = 0;
    constexpr int kRightHand = 1;

    CallTriggerHapticPulse(openVR, isLeft ? kLeftHand : kRightHand, Config::options.latchHapticDuration);
}

ClimbManager* ClimbManager::GetSingleton()
{
    static ClimbManager instance;
    return &instance;
}

void ClimbManager::Initialize()
{
    if (m_initialized) {
        spdlog::warn("ClimbManager already initialized");
        return;
    }

    // Register for HIGGS PrePhysicsStep callback (synchronized with physics)
    if (g_higgsInterface) {
        g_higgsInterface->AddPrePhysicsStepCallback(&ClimbManager::OnPrePhysicsStep);
        spdlog::info("ClimbManager: Registered PrePhysicsStep callback with HIGGS");
    } else {
        spdlog::error("ClimbManager: HIGGS interface not available, climbing won't work");
        return;
    }

    // Initialize HIGGS compatibility manager
    HiggsCompatManager::GetSingleton()->Initialize();


    // Register for grip button input
    auto* inputMgr = InputManager::GetSingleton();
    if (inputMgr && inputMgr->IsInitialized()) {
        uint64_t gripMask = vr::ButtonMaskFromId(vr::k_EButton_Grip);
        m_gripCallbackId = inputMgr->AddVrButtonCallback(gripMask,
            [this](bool isLeft, bool isReleased, vr::EVRButtonId buttonId) -> bool {
                if (isReleased) {
                    return OnGripReleased(isLeft);
                } else {
                    return OnGripPressed(isLeft);
                }
            });
    } else {
        spdlog::warn("ClimbManager: InputManager not available, grip input won't work");
    }

    m_initialized = true;
    spdlog::info("ClimbManager initialized");
}

void ClimbManager::Shutdown()
{
    if (!m_initialized) {
        return;
    }

    // Stop any active climbing
    if (m_leftGrabbing) {
        StopClimb(true);
    }
    if (m_rightGrabbing) {
        StopClimb(false);
    }

    auto* inputMgr = InputManager::GetSingleton();
    if (inputMgr && m_gripCallbackId != InputManager::InvalidCallbackId) {
        inputMgr->RemoveVrButtonCallback(m_gripCallbackId);
        m_gripCallbackId = InputManager::InvalidCallbackId;
    }

    m_initialized = false;
    spdlog::info("ClimbManager shut down");
}

void ClimbManager::OnPrePhysicsStep(void* world)
{
    // This is called every physics step, synchronized with the simulation
    auto* instance = GetSingleton();
    if (!instance->m_initialized) {
        return;
    }

    // Hot reload config if enabled (throttled to every ~90 frames)
    static uint32_t reloadCounter = 0;
    if (Config::options.hotReloadEnabled && (++reloadCounter % 90 == 0)) {
        Config::ReloadIfModified();
    }

    // Calculate deltaTime for this frame
    static auto lastTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float deltaTime = std::chrono::duration<float>(now - lastTime).count();
    lastTime = now;

    // Clamp deltaTime
    if (deltaTime > 0.1f) deltaTime = 0.1f;

    // Update ballistic controller if in flight
    auto* ballistic = BallisticController::GetSingleton();
    if (ballistic->IsInFlight()) {
        if (deltaTime > 0.0f) {
            bool stillFlying = ballistic->Update(deltaTime);

            // If flight ended, check if it was due to auto-catch
            if (!stillFlying) {
                auto catchResult = ballistic->GetAutoCatchResult();
                if (catchResult != BallisticController::AutoCatchHand::kNone) {
                    instance->HandleAutoCatch(static_cast<uint8_t>(catchResult));
                    ballistic->ClearAutoCatchResult();
                }
            }
        }
    }
    // Post-flight autocatch: check for catch opportunities during grace period
    // This handles the case where player lands/bumps into something before grabbing
    else if (ballistic->IsInAutoCatchWindow() && !instance->IsClimbing()) {
        auto catchResult = ballistic->CheckAutoCatch();
        if (catchResult != BallisticController::AutoCatchHand::kNone) {
            instance->HandleAutoCatch(static_cast<uint8_t>(catchResult));
        }
    }

    // Update CriticalStrikeManager (must run even after landing to handle slow-mo timeout)
    static uint32_t criticalFrameCount = 0;
    CriticalStrikeManager::GetSingleton()->Update(++criticalFrameCount);

    // Update ClimbExitCorrector (must run even after landing to complete smooth correction)
    ClimbExitCorrector::GetSingleton()->Update(deltaTime);

    // Update HiggsCompatManager (handles delayed restore of HIGGS settings)
    HiggsCompatManager::GetSingleton()->Update();

    instance->UpdateClimbing();

    // Keep setting InAir state while climbing (gravity disabled)
    if (instance->m_gravityDisabled) {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (player) {
            auto* controller = player->GetCharController();
            if (controller) {
                controller->wantState = RE::hkpCharacterStateType::kInAir;
                // Mark as unsupported so physics doesn't apply ground movement
                controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kUnsupported;
            }
        }
    }
}

bool ClimbManager::OnGripPressed(bool isLeft)
{
    // Don't process climbing inputs while in menus
    if (MenuChecker::IsGameStopped()) {
        return false;
    }

    spdlog::trace("ClimbManager: Grip pressed ({})", isLeft ? "left" : "right");

    // Track grip state for auto-catch feature
    if (isLeft) {
        m_leftGripHeld = true;
    } else {
        m_rightGripHeld = true;
    }

    // If either hand is already climbing, consume ALL grip inputs to prevent
    // the game from grabbing items while we're holding onto surfaces
    bool eitherHandClimbing = m_leftGrabbing || m_rightGrabbing;

    // Check if there's a climbable surface near this hand
    if (!ClimbSurfaceDetector::CanGrabSurface(isLeft)) {
        // No surface nearby - don't start climbing with this hand
        // But still consume input if other hand is climbing
        return eitherHandClimbing;
    }

    // Check if climbing is enabled (beast forms can always climb)
    if (!Config::options.climbingEnabled && !IsPlayerInBeastForm()) {
        return eitherHandClimbing;
    }

    // Check if player has enough stamina to climb
    if (!StaminaDrainManager::GetSingleton()->CanStartClimbing()) {
        spdlog::debug("ClimbManager: Cannot climb - no stamina");
        // Still consume if other hand is climbing
        return eitherHandClimbing;
    }

    spdlog::info("ClimbManager: Starting climb ({}) - surface detected", isLeft ? "left" : "right");
    StartClimb(isLeft);

    // Consume input - we're climbing, don't let HIGGS grab objects
    return true;
}

bool ClimbManager::OnGripReleased(bool isLeft)
{
    // Track grip state for auto-catch feature
    if (isLeft) {
        m_leftGripHeld = false;
    } else {
        m_rightGripHeld = false;
    }

    // Check if this hand was climbing
    bool wasClimbing = isLeft ? m_leftGrabbing : m_rightGrabbing;

    // Check if either hand is currently climbing (before we process this release)
    bool eitherHandClimbing = m_leftGrabbing || m_rightGrabbing;

    if (wasClimbing) {
        spdlog::info("ClimbManager: Grip released ({})", isLeft ? "left" : "right");
        StopClimb(isLeft);
    }

    // Consume input if either hand was/is climbing
    // This prevents the game from processing grip releases during climb
    return eitherHandClimbing;
}

void ClimbManager::StartClimb(bool isLeft)
{
    bool wasClimbing = m_leftGrabbing || m_rightGrabbing;
    bool alreadyThisHandGrabbing = isLeft ? m_leftGrabbing : m_rightGrabbing;

    // If we're currently in ballistic flight, abort it - player is grabbing mid-air
    auto* ballistic = BallisticController::GetSingleton();
    if (ballistic->IsInFlight()) {
        spdlog::info("ClimbManager: Aborting flight - grabbed surface mid-air");
        ballistic->Abort();
    }

    // Cancel exit correction if player re-grabs
    ballistic->CancelExitCorrection();

    // End slow-mo and cancel any position correction when player starts climbing
    CriticalStrikeManager::GetSingleton()->OnClimbStart();
    ClimbExitCorrector::GetSingleton()->Cancel();

    // Clear safe position tracking for this new climb session
    // This ensures we don't teleport back to an old position from a previous climb
    if (!wasClimbing) {
        ClimbExitCorrector::GetSingleton()->ClearSafePosition();
    }

    // Log climb mode entry
    if (!wasClimbing) {
        spdlog::info("=== CLIMB MODE: ENTER ({} hand) ===", isLeft ? "left" : "right");
    } else {
        spdlog::info("ClimbManager: {} hand grabbed (already climbing)", isLeft ? "Left" : "Right");
    }

    // Play grip sound whenever a hand grabs a surface (both initial grab and subsequent grabs)
    AudioManager::GetSingleton()->PlayGripSound(IsPlayerInBeastForm());

    // Haptic feedback when hand latches (only when this hand transitions to grabbing)
    if (!alreadyThisHandGrabbing && Config::options.latchHapticsEnabled) {
        TriggerLatchHaptic(isLeft);
    }

    RE::NiPoint3 handPos = GetHandWorldPosition(isLeft);

    // Get player position to calculate hand offset
    auto* player = RE::PlayerCharacter::GetSingleton();
    RE::NiPoint3 playerPos = player ? player->GetPosition() : RE::NiPoint3{0.0f, 0.0f, 0.0f};
    RE::NiPoint3 handOffset = handPos - playerPos;

    if (isLeft) {
        m_leftGrabbing = true;
        m_leftGrabPoint = handPos;
        m_leftPrevHandOffset = handOffset;  // Store offset, not world pos
    } else {
        m_rightGrabbing = true;
        m_rightGrabPoint = handPos;
        m_rightPrevHandOffset = handOffset;  // Store offset, not world pos
    }

    // Initialize smoothing target to current player position
    if (!m_hasTargetPosition) {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (player) {
            m_targetPosition = player->GetPosition();
            m_hasTargetPosition = true;
            m_lastUpdateTime = std::chrono::steady_clock::now();
            m_hasLastUpdateTime = true;
        }
    }

    // Disable gravity and set to "in air" state when starting to climb
    if (!m_gravityDisabled && (m_leftGrabbing || m_rightGrabbing)) {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (player) {
            auto* controller = player->GetCharController();
            if (controller) {
                m_savedGravity = controller->gravity;
                controller->gravity = 0.0f;
                m_gravityDisabled = true;

                // Notify ClimbingDamageManager that we started climbing
                ClimbingDamageManager::GetSingleton()->SetClimbingState(true);

                // Disable HIGGS gravity gloves while climbing
                HiggsCompatManager::GetSingleton()->DisableGravityGloves();

                spdlog::info("ClimbManager: Gravity disabled (was {})", m_savedGravity);
            }
        }
    }
}

void ClimbManager::StopClimb(bool isLeft)
{
    if (isLeft) {
        m_leftGrabbing = false;
    } else {
        m_rightGrabbing = false;
    }

    // When completely releasing (no hands grabbing), calculate and apply launch
    if (!m_leftGrabbing && !m_rightGrabbing) {
        spdlog::info("=== CLIMB MODE: EXIT ({} hand released) ===", isLeft ? "left" : "right");

        // Notify stamina manager that climbing stopped
        StaminaDrainManager::GetSingleton()->OnClimbingStopped();

        // Notify damage manager that climbing stopped
        ClimbingDamageManager::GetSingleton()->SetClimbingState(false);

        // Restore HIGGS gravity gloves
        HiggsCompatManager::GetSingleton()->RestoreGravityGloves();

        // Calculate launch velocity from movement history
        RE::NiPoint3 launchVelocity = CalculateLaunchVelocity();
        float speed = std::sqrt(launchVelocity.x * launchVelocity.x +
                                launchVelocity.y * launchVelocity.y +
                                launchVelocity.z * launchVelocity.z);

        // Use BallisticController for both launch and fall scenarios
        // This ensures auto-catch works in both cases
        auto* ballistic = BallisticController::GetSingleton();

        if (speed >= Config::options.minLaunchSpeed) {
            // Fast release = launch with velocity
            ballistic->Launch(launchVelocity, m_savedGravity);
            spdlog::info("ClimbManager: Launching with velocity ({}, {}, {}), speed: {}, gravity: {}",
                         launchVelocity.x, launchVelocity.y, launchVelocity.z, speed, m_savedGravity);
        } else {
            // Slow release = controlled fall (still has auto-catch protection)
            ballistic->StartFall(m_savedGravity);
            spdlog::info("ClimbManager: Starting fall (speed {} below threshold {})",
                         speed, Config::options.minLaunchSpeed);
        }

        // This prevents correcting while we're intentionally ignoring small obstacles
        ballistic->RequestExitCorrection();

        // BallisticController now manages gravity
        m_gravityDisabled = false;

        // Clear velocity history
        m_velocityHistory.clear();

        // Reset smoothing state
        m_hasTargetPosition = false;
        m_hasLastUpdateTime = false;
    }
}

void ClimbManager::UpdateClimbing()
{
    if (!m_leftGrabbing && !m_rightGrabbing) {
        return;  // Not climbing
    }

    // Track safe positions for exit correction fallback (every 50 frames)
    ClimbExitCorrector::GetSingleton()->UpdateSafePositionCheck();

    // Force release grips if a game-stopping menu opened (dialogue, pause, etc.)
    // Use NoLaunch version to avoid starting ballistic flight during menu
    if (MenuChecker::IsGameStopped()) {
        spdlog::info("ClimbManager: Menu opened while climbing - releasing all grips");
        ForceReleaseAllGripsNoLaunch();
        return;
    }

    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    // Calculate deltaTime
    auto now = std::chrono::steady_clock::now();
    float deltaTime = 0.0f;
    if (m_hasLastUpdateTime) {
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - m_lastUpdateTime);
        deltaTime = elapsed.count() / 1000000.0f;  // Convert to seconds
    }
    m_lastUpdateTime = now;
    m_hasLastUpdateTime = true;

    // Validate deltaTime
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0f) {
        return;  // Skip this frame
    }

    // Update stamina drain - if out of stamina, force release
    if (!StaminaDrainManager::GetSingleton()->UpdateClimbingDrain(deltaTime)) {
        spdlog::info("ClimbManager: Out of stamina! Forcing release of all grips");
        if (m_leftGrabbing) {
            StopClimb(true);
        }
        if (m_rightGrabbing) {
            StopClimb(false);
        }
        return;  // Don't process climbing this frame
    }
    // Cap deltaTime to avoid excessive jumps after pauses
    constexpr float MAX_DELTA = 0.1f;
    if (deltaTime > MAX_DELTA) {
        deltaTime = MAX_DELTA;
    }

    RE::NiPoint3 playerPos = player->GetPosition();
    RE::NiPoint3 totalDelta{0.0f, 0.0f, 0.0f};
    int grabCount = 0;

    // Calculate movement delta from left hand using OFFSETS from player
    // This ensures player movement doesn't affect the delta calculation
    if (m_leftGrabbing) {
        RE::NiPoint3 currentHandPos = GetHandWorldPosition(true);
        RE::NiPoint3 currentHandOffset = currentHandPos - playerPos;
        // Delta is how much the hand offset changed
        // Player should move in the OPPOSITE direction (pulling yourself up)
        RE::NiPoint3 handMovement = currentHandOffset - m_leftPrevHandOffset;
        totalDelta = totalDelta - handMovement;  // Negative = pull toward grab point
        m_leftPrevHandOffset = currentHandOffset;
        grabCount++;
    }

    // Calculate movement delta from right hand using OFFSETS from player
    if (m_rightGrabbing) {
        RE::NiPoint3 currentHandPos = GetHandWorldPosition(false);
        RE::NiPoint3 currentHandOffset = currentHandPos - playerPos;
        RE::NiPoint3 handMovement = currentHandOffset - m_rightPrevHandOffset;
        totalDelta = totalDelta - handMovement;
        m_rightPrevHandOffset = currentHandOffset;
        grabCount++;
    }

    // Average if both hands are grabbing
    if (grabCount > 1) {
        totalDelta.x /= static_cast<float>(grabCount);
        totalDelta.y /= static_cast<float>(grabCount);
        totalDelta.z /= static_cast<float>(grabCount);
    }

    // Track velocity for launch mechanics
    // Store the movement delta and time for velocity calculation
    VelocitySample sample;
    sample.delta = totalDelta;
    sample.deltaTime = deltaTime;
    m_velocityHistory.push_back(sample);

    // Trim old samples (keep only recent ones within time window)
    float totalTime = 0.0f;
    while (m_velocityHistory.size() > static_cast<size_t>(Config::options.maxVelocitySamples)) {
        m_velocityHistory.pop_front();
    }
    // Also trim by time - remove samples older than velocityHistoryTime
    while (!m_velocityHistory.empty()) {
        totalTime = 0.0f;
        for (const auto& s : m_velocityHistory) {
            totalTime += s.deltaTime;
        }
        if (totalTime > Config::options.velocityHistoryTime && m_velocityHistory.size() > 1) {
            m_velocityHistory.pop_front();
        } else {
            break;
        }
    }

    // Accumulate delta into target position
    m_targetPosition.x += totalDelta.x;
    m_targetPosition.y += totalDelta.y;
    m_targetPosition.z += totalDelta.z;

    // Apply smoothed movement toward target
    ApplyClimbMovement(deltaTime);
}

void ClimbManager::ApplyClimbMovement(float deltaTime)
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        return;
    }

    // Get current player position
    RE::NiPoint3 currentPos = player->GetPosition();

    // Wall collision detection - clamp TARGET position directly so smoothing handles the transition
    // Check from HMD toward target position (not just this frame's movement)
    RE::NiAVObject* hmdNode = VRNodes::GetHMD();
    RE::NiPoint3 hmdPos = hmdNode ? hmdNode->world.translate : currentPos;

    // Vector from current position to target
    RE::NiPoint3 toTarget = m_targetPosition - currentPos;
    float targetDistance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y + toTarget.z * toTarget.z);

    if (targetDistance > 0.001f) {
        // Normalize direction to target
        RE::NiPoint3 targetDir = {
            toTarget.x / targetDistance,
            toTarget.y / targetDistance,
            toTarget.z / targetDistance
        };

        // Calculate directional buffer based on HMD forward
        // Forward needs larger buffer (16.0) to prevent near-plane culling
        // Other directions can use smaller buffer (8.0)
        constexpr float BUFFER_FORWARD = 16.0f;
        constexpr float BUFFER_OTHER = 14.0f;
        float wallBuffer = BUFFER_OTHER;

        if (hmdNode) {
            // Get HMD forward vector (Y-axis in Skyrim's coordinate system)
            const auto& rotation = hmdNode->world.rotate;
            RE::NiPoint3 hmdForward = {
                rotation.entry[0][1],
                rotation.entry[1][1],
                rotation.entry[2][1]
            };

            // Dot product: 1.0 = moving forward, -1.0 = moving backward, 0 = sideways
            float forwardDot = targetDir.x * hmdForward.x + targetDir.y * hmdForward.y + targetDir.z * hmdForward.z;

            // Scale buffer: BUFFER_OTHER for backward/sideways, BUFFER_FORWARD for forward
            float forwardAmount = (std::max)(0.0f, forwardDot);
            wallBuffer = BUFFER_OTHER + forwardAmount * (BUFFER_FORWARD - BUFFER_OTHER);
        }

        // Cast ray from HMD toward target position
        float allowedDist = Raycast::GetAllowedDistance(hmdPos, targetDir, targetDistance, wallBuffer, IsSolidWorldLayer);

        if (allowedDist < targetDistance) {
            // Wall detected - clamp target position directly (no pressure buildup)
            m_targetPosition.x = currentPos.x + targetDir.x * allowedDist;
            m_targetPosition.y = currentPos.y + targetDir.y * allowedDist;
            m_targetPosition.z = currentPos.z + targetDir.z * allowedDist;
        }
    }

    // Exponential smoothing - smoothFactor = 1 - exp(-speed * deltaTime)
    float smoothFactor = 1.0f - std::exp(-Config::options.smoothingSpeed * deltaTime);

    // Clamp to valid range [0, 1]
    if (smoothFactor < 0.0f) smoothFactor = 0.0f;
    if (smoothFactor > 1.0f) smoothFactor = 1.0f;

    // Smooth toward (possibly clamped) target - single smoothing handles everything
    RE::NiPoint3 newPos;
    newPos.x = currentPos.x + (m_targetPosition.x - currentPos.x) * smoothFactor;
    newPos.y = currentPos.y + (m_targetPosition.y - currentPos.y) * smoothFactor;
    newPos.z = currentPos.z + (m_targetPosition.z - currentPos.z) * smoothFactor;

    // Floor detection - prevent climbing down into ground
    // Cast ray from player position + offset, downward for 5 units
    // If we hit ground and we're trying to move down, block the downward component
    if (newPos.z < currentPos.z) {  // Only check if moving downward
        constexpr float FLOOR_CHECK_HEIGHT = 80.0f;  // Start ray this high above player origin
        constexpr float FLOOR_CHECK_DISTANCE = 5.0f; // Check this far down

        RE::NiPoint3 floorCheckOrigin = currentPos;
        floorCheckOrigin.z += FLOOR_CHECK_HEIGHT;
        RE::NiPoint3 downDir = { 0.0f, 0.0f, -1.0f };

        float floorDist = Raycast::GetAllowedDistance(floorCheckOrigin, downDir, FLOOR_CHECK_DISTANCE, 0.0f, IsSolidWorldLayer);

        if (floorDist < FLOOR_CHECK_DISTANCE) {
            // Ground detected below - prevent further downward movement
            newPos.z = currentPos.z;
            m_targetPosition.z = currentPos.z;
        }
    }

    // Zero out velocity to prevent physics drift
    RE::hkVector4 zero;
    zero.quad.m128_f32[0] = 0.0f;
    zero.quad.m128_f32[1] = 0.0f;
    zero.quad.m128_f32[2] = 0.0f;
    zero.quad.m128_f32[3] = 0.0f;
    controller->SetLinearVelocityImpl(zero);

    // Apply new position
    player->SetPosition(newPos, true);
}

RE::NiPoint3 ClimbManager::GetHandWorldPosition(bool isLeft) const
{
    RE::NiAVObject* handNode = isLeft ? VRNodes::GetLeftHand() : VRNodes::GetRightHand();

    if (handNode) {
        return handNode->world.translate;
    }

    // Fallback: return player position if hand not available
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (player) {
        return player->GetPosition();
    }

    return RE::NiPoint3{0.0f, 0.0f, 0.0f};
}

bool ClimbManager::IsPlayerInBeastForm()
{
    return EquipmentManager::GetSingleton()->IsInBeastForm();
}

RE::NiPoint3 ClimbManager::CalculateLaunchVelocity() const
{
    if (m_velocityHistory.empty()) {
        return RE::NiPoint3{0.0f, 0.0f, 0.0f};
    }

    // Velocity-weighted average: faster hand movements have more influence on final direction
    // Weight = pow(speed, exponent) where exponent is configurable
    // This makes the "flick" at release dominate over slow positioning movements
    float exponent = Config::options.velocityWeightExponent;
    float filterAngle = Config::options.launchDirectionFilter;
    bool filterEnabled = filterAngle > 0.0f && filterAngle < 180.0f;

    // Convert angle threshold to cosine (for dot product comparison)
    // cos(60°) ≈ 0.5, cos(90°) = 0, cos(45°) ≈ 0.707
    constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
    float cosThreshold = filterEnabled ? std::cos(filterAngle * DEG_TO_RAD) : -1.0f;

    // ===== FIRST PASS: Calculate velocity-weighted dominant direction =====
    RE::NiPoint3 weightedDir{0.0f, 0.0f, 0.0f};
    float dirWeight = 0.0f;

    for (const auto& sample : m_velocityHistory) {
        if (sample.deltaTime <= 0.0f) continue;

        // Calculate instantaneous velocity
        RE::NiPoint3 sampleVel;
        sampleVel.x = sample.delta.x / sample.deltaTime;
        sampleVel.y = sample.delta.y / sample.deltaTime;
        sampleVel.z = sample.delta.z / sample.deltaTime;

        float speed = std::sqrt(sampleVel.x * sampleVel.x +
                                sampleVel.y * sampleVel.y +
                                sampleVel.z * sampleVel.z);

        if (speed < 0.001f) continue;  // Skip near-zero velocity

        // Normalize direction and weight by speed^exponent
        float weight = std::pow(speed, exponent);
        weightedDir.x += (sampleVel.x / speed) * weight;
        weightedDir.y += (sampleVel.y / speed) * weight;
        weightedDir.z += (sampleVel.z / speed) * weight;
        dirWeight += weight;
    }

    // Normalize to get dominant direction
    RE::NiPoint3 dominantDir{0.0f, 0.0f, 0.0f};
    if (dirWeight > 0.0f) {
        float dirMag = std::sqrt(weightedDir.x * weightedDir.x +
                                 weightedDir.y * weightedDir.y +
                                 weightedDir.z * weightedDir.z);
        if (dirMag > 0.001f) {
            dominantDir.x = weightedDir.x / dirMag;
            dominantDir.y = weightedDir.y / dirMag;
            dominantDir.z = weightedDir.z / dirMag;
        }
    }

    // ===== SECOND PASS: Accumulate velocity, rejecting outliers =====
    RE::NiPoint3 weightedVelocity{0.0f, 0.0f, 0.0f};
    float totalWeight = 0.0f;
    int rejectedSamples = 0;

    for (const auto& sample : m_velocityHistory) {
        if (sample.deltaTime <= 0.0f) continue;

        // Calculate instantaneous velocity
        RE::NiPoint3 sampleVel;
        sampleVel.x = sample.delta.x / sample.deltaTime;
        sampleVel.y = sample.delta.y / sample.deltaTime;
        sampleVel.z = sample.delta.z / sample.deltaTime;

        float speed = std::sqrt(sampleVel.x * sampleVel.x +
                                sampleVel.y * sampleVel.y +
                                sampleVel.z * sampleVel.z);

        if (speed < 0.001f) continue;

        // Direction filter: reject samples that deviate too much from dominant direction
        if (filterEnabled) {
            // Normalize sample direction
            RE::NiPoint3 sampleDir;
            sampleDir.x = sampleVel.x / speed;
            sampleDir.y = sampleVel.y / speed;
            sampleDir.z = sampleVel.z / speed;

            // Dot product = cos(angle between vectors)
            float dot = sampleDir.x * dominantDir.x +
                        sampleDir.y * dominantDir.y +
                        sampleDir.z * dominantDir.z;

            if (dot < cosThreshold) {
                rejectedSamples++;
                continue;  // Outside allowed cone, skip this sample
            }
        }

        // Weight = speed^exponent
        float weight = std::pow(speed, exponent);

        // Accumulate weighted velocity
        weightedVelocity.x += sampleVel.x * weight;
        weightedVelocity.y += sampleVel.y * weight;
        weightedVelocity.z += sampleVel.z * weight;
        totalWeight += weight;
    }

    if (totalWeight <= 0.0f) {
        return RE::NiPoint3{0.0f, 0.0f, 0.0f};
    }

    // Normalize by total weight to get weighted average velocity
    RE::NiPoint3 avgVelocity;
    avgVelocity.x = weightedVelocity.x / totalWeight;
    avgVelocity.y = weightedVelocity.y / totalWeight;
    avgVelocity.z = weightedVelocity.z / totalWeight;

    // Calculate launch multiplier and max speed based on equipment and race
    auto* equipMgr = EquipmentManager::GetSingleton();
    bool isBeast = equipMgr->IsInBeastForm();
    float multiplier;
    float maxSpeed;
    const char* modeStr = "";

    if (isBeast) {
        // Beast form: use beast values directly (no weight penalty)
        multiplier = Config::options.beastLaunchMultiplier;
        maxSpeed = Config::options.beastMaxLaunchSpeed;
        modeStr = " (BEAST)";
    } else if (Config::options.overencumberedEnabled && equipMgr->IsOverEncumbered()) {
        // Overencumbered: use overencumbered values
        multiplier = Config::options.minLaunchMultiplier;  // Use min multiplier
        maxSpeed = Config::options.overencumberedMaxLaunchSpeed;
        modeStr = " (OVERENC)";
    } else if (Config::options.weightScalingEnabled) {
        // Weight-based scaling (uses skill-scaled weight)
        float rawWeight = equipMgr->GetTotalArmorWeight();
        float skillScaledWeight = equipMgr->GetTotalArmorWeightSkillScaled();
        float maxWeight = Config::options.maxArmorWeight;
        float weightRatio = std::clamp(skillScaledWeight / maxWeight, 0.0f, 1.0f);

        // Interpolate multiplier
        multiplier = Config::options.maxLaunchMultiplier -
                    (Config::options.maxLaunchMultiplier - Config::options.minLaunchMultiplier) * weightRatio;

        // Interpolate max speed
        maxSpeed = Config::options.baseLaunchSpeed -
                  (Config::options.baseLaunchSpeed - Config::options.minLaunchSpeedWeighted) * weightRatio;

        // Add race bonuses
        float raceBonus = 0.0f;
        if (equipMgr->IsKhajiit()) {
            raceBonus = Config::options.khajiitMaxLaunchSpeedBonus;
            maxSpeed += raceBonus;
            modeStr = " (KHAJIIT)";
        } else if (equipMgr->IsArgonian()) {
            raceBonus = Config::options.argonianMaxLaunchSpeedBonus;
            maxSpeed += raceBonus;
            modeStr = " (ARGONIAN)";
        }

        spdlog::trace("Weight calc: raw={:.1f} scaled={:.1f} ratio={:.2f} | mult={:.2f} maxSpd={:.0f} raceBonus={:.0f}",
            rawWeight, skillScaledWeight, weightRatio, multiplier, maxSpeed, raceBonus);
    } else {
        // No weight scaling - use base values
        multiplier = Config::options.maxLaunchMultiplier;
        maxSpeed = Config::options.baseLaunchSpeed;

        // Add race bonuses
        if (equipMgr->IsKhajiit()) {
            maxSpeed += Config::options.khajiitMaxLaunchSpeedBonus;
            modeStr = " (KHAJIIT)";
        } else if (equipMgr->IsArgonian()) {
            maxSpeed += Config::options.argonianMaxLaunchSpeedBonus;
            modeStr = " (ARGONIAN)";
        }
    }

    // Apply multipliers: horizontal gets extra boost, vertical uses base multiplier
    RE::NiPoint3 velocity;
    velocity.x = avgVelocity.x * multiplier * Config::options.horizontalLaunchBoost;
    velocity.y = avgVelocity.y * multiplier * Config::options.horizontalLaunchBoost;
    velocity.z = avgVelocity.z * multiplier;

    // Clamp to maximum launch speed
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
    if (speed > maxSpeed) {
        float scale = maxSpeed / speed;
        velocity.x *= scale;
        velocity.y *= scale;
        velocity.z *= scale;
        spdlog::info("Launch velocity clamped{}: ({:.1f}, {:.1f}, {:.1f}) [was {:.1f} u/s, cap {:.1f}] exp={:.1f} filter={:.0f}° rejected={}",
            modeStr, velocity.x, velocity.y, velocity.z, speed, maxSpeed, exponent, filterAngle, rejectedSamples);
    } else {
        spdlog::info("Launch velocity{}: ({:.1f}, {:.1f}, {:.1f}), speed={:.1f}, samples={}, exp={:.1f}, filter={:.0f}°, rejected={}",
            modeStr,
            velocity.x, velocity.y, velocity.z, speed,
            m_velocityHistory.size(), exponent, filterAngle, rejectedSamples);
    }

    return velocity;
}

void ClimbManager::ApplyLaunch(const RE::NiPoint3& velocity)
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        return;
    }

    // Convert velocity to Havok scale
    float havokScale = RE::bhkWorld::GetWorldScale();

    RE::hkVector4 hkVelocity;
    hkVelocity.quad.m128_f32[0] = velocity.x * havokScale;
    hkVelocity.quad.m128_f32[1] = velocity.y * havokScale;
    hkVelocity.quad.m128_f32[2] = velocity.z * havokScale;
    hkVelocity.quad.m128_f32[3] = 0.0f;

    controller->SetLinearVelocityImpl(hkVelocity);
}

void ClimbManager::HandleAutoCatch(uint8_t catchResult)
{
    // Auto-catch triggered: hands detected a surface below while falling
    // If grip is held on the corresponding hand, start climbing immediately
    // Otherwise, the player needs to press grip to grab

    bool leftCatch = (catchResult & BallisticController::AutoCatchHand::kLeft) != 0;
    bool rightCatch = (catchResult & BallisticController::AutoCatchHand::kRight) != 0;

    // Check if grip is already held - if so, start climbing immediately
    // But only if player has stamina to climb
    bool canClimb = StaminaDrainManager::GetSingleton()->CanStartClimbing();

    if (leftCatch && m_leftGripHeld && !m_leftGrabbing && canClimb) {
        spdlog::info("ClimbManager: Auto-catch -> left hand grab (grip was held)");
        StartClimb(true);
    }

    if (rightCatch && m_rightGripHeld && !m_rightGrabbing && canClimb) {
        spdlog::info("ClimbManager: Auto-catch -> right hand grab (grip was held)");
        StartClimb(false);
    }

}

void ClimbManager::ForceReleaseAllGrips()
{
    // Force release both hands - called by external systems like ClimbingDamageManager
    // when player takes significant damage while climbing
    spdlog::info("ClimbManager: Force releasing all grips");

    if (m_leftGrabbing) {
        StopClimb(true);
    }
    if (m_rightGrabbing) {
        StopClimb(false);
    }
}

void ClimbManager::ForceReleaseAllGripsNoLaunch()
{
    // Force release both hands WITHOUT starting ballistic flight
    // Used when menu opens - we just want to cleanly exit climbing state
    spdlog::info("ClimbManager: Force releasing all grips (no launch)");

    bool wasClimbing = m_leftGrabbing || m_rightGrabbing;

    // Reset grab states
    if (m_leftGrabbing) {
        m_leftGrabbing = false;
    }
    if (m_rightGrabbing) {
        m_rightGrabbing = false;
    }

    if (wasClimbing) {
        // Notify managers
        StaminaDrainManager::GetSingleton()->OnClimbingStopped();
        ClimbingDamageManager::GetSingleton()->SetClimbingState(false);

        // Restore HIGGS settings
        HiggsCompatManager::GetSingleton()->RestoreGravityGloves();

        // Restore gravity directly (no ballistic flight)
        if (m_gravityDisabled) {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                auto* controller = player->GetCharController();
                if (controller) {
                    controller->gravity = m_savedGravity;

                    // Reset to grounded state
                    controller->wantState = RE::hkpCharacterStateType::kOnGround;
                    controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kSupported;

                    // Zero velocity
                    RE::hkVector4 zero;
                    zero.quad.m128_f32[0] = 0.0f;
                    zero.quad.m128_f32[1] = 0.0f;
                    zero.quad.m128_f32[2] = 0.0f;
                    zero.quad.m128_f32[3] = 0.0f;
                    controller->SetLinearVelocityImpl(zero);
                }
            }
            m_gravityDisabled = false;
        }

        // Clear velocity history
        m_velocityHistory.clear();

        // Reset smoothing state
        m_hasTargetPosition = false;
        m_hasLastUpdateTime = false;

        spdlog::info("ClimbManager: Cleanly exited climbing state (gravity restored to {})", m_savedGravity);
    }
}
