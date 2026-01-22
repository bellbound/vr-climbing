#include "BallisticController.h"
#include "ClimbManager.h"
#include "ClimbSurfaceDetector.h"
#include "CriticalStrikeManager.h"
#include "ClimbExitCorrector.h"
#include "AudioManager.h"
#include "Config.h"
#include "util/Raycast.h"
#include "util/VRNodes.h"
#include <RE/B/bhkCharProxyController.h>
#include <RE/H/hkpCharacterProxy.h>
#include <spdlog/spdlog.h>
#include <cmath>

// Frame counter for CriticalStrikeManager throttling
static uint32_t s_frameCount = 0;

// Layer filter - only land on solid world geometry
static bool IsGroundLayer(RE::COL_LAYER layer)
{
    switch (layer) {
    case RE::COL_LAYER::kStatic:
    case RE::COL_LAYER::kAnimStatic:  // Can land on moving platforms
    case RE::COL_LAYER::kTerrain:
    case RE::COL_LAYER::kGround:
    case RE::COL_LAYER::kTrees:       // Can land on tree branches
    case RE::COL_LAYER::kProps:       // Can land on furniture/props
        return true;
    default:
        return false;
    }
}

BallisticController* BallisticController::GetSingleton()
{
    static BallisticController instance;
    return &instance;
}

void BallisticController::Launch(const RE::NiPoint3& velocity, float gravity)
{
    if (m_inFlight) {
        spdlog::warn("BallisticController: Already in flight, ignoring new launch");
        return;
    }

    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        return;
    }

    // Store the original gravity to restore on landing
    // Note: gravity param is the saved value from ClimbManager (before it was zeroed)
    m_savedGravity = gravity;

    if (regularPhysics) {
        m_gravity = gravity * 700.0f;
        controller->gravity = gravity;
    }
    else
    {
        // Keep gravity disabled - we'll apply it ourselves as a delta
        controller->gravity = 0.0f;

        // Convert game's gravity to usable units/s²
        // Skyrim's gravity is stored as a positive value (e.g., ~1.35)
        m_gravity = gravity * 700.0f;  // Scale factor to convert to usable units/s²
    }





    // Apply launch velocity as a one-time impulse (add to current velocity)
    float havokScale = RE::bhkWorld::GetWorldScale();
    RE::hkVector4 currentVel;
    controller->GetLinearVelocityImpl(currentVel);

    RE::hkVector4 launchImpulse;
    launchImpulse.quad.m128_f32[0] = currentVel.quad.m128_f32[0] + velocity.x * havokScale;
    launchImpulse.quad.m128_f32[1] = currentVel.quad.m128_f32[1] + velocity.y * havokScale;
    launchImpulse.quad.m128_f32[2] = currentVel.quad.m128_f32[2] + velocity.z * havokScale;
    launchImpulse.quad.m128_f32[3] = 0.0f;
    controller->SetLinearVelocityImpl(launchImpulse);

    m_flightTime = 0.0f;
    m_inFlight = true;
    m_launchVelocity = velocity;  // Store for exit correction speed checks																			 
    m_autoCatchResult = AutoCatchHand::kNone;  // Clear any previous auto-catch result
    // Note: m_needsExitCorrection is set by ClimbManager via RequestExitCorrection()

    // Initialize clearance tracking - check if we're starting with ground contact
    m_hadInitialSupport = (controller->supportBody.get() != nullptr);
    m_hasCleared = !m_hadInitialSupport;  // If no initial support, we're already "cleared"
    m_clearanceTimer = 0.0f;

    // Notify CriticalStrikeManager
    CriticalStrikeManager::GetSingleton()->OnLaunchStart();

    // Calculate launch speed for ghost mode check
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);

    // Conditional ghost mode: only if speed is high enough AND path is clear
    m_ghostModeActive = false;
    m_ghostModeTimer = 0.0f;
    m_ghostModeDuration = 0.0f;

    if (ShouldEnterGhostMode(velocity, speed)) {
        SetPlayerWorldCollision(false);
        m_ghostModeActive = true;
        m_ghostModeDuration = Config::options.ghostModeDuration;
        m_ghostModeTimer = m_ghostModeDuration;
    }

    // Play launch sound based on speed and beast form
    bool isBeast = ClimbManager::IsPlayerInBeastForm();
    AudioManager::GetSingleton()->PlayLaunchSound(speed, isBeast);

    if (isBeast)
    {
        if (Config::options.regularPhysicsOnFallBeast)
        {
            regularPhysics = true;
        }
        else 
        {
            regularPhysics = false;
        }
    }
    else
    {
        if (Config::options.regularPhysicsOnFall)
        {
            regularPhysics = true;
        }
        else
        {
            regularPhysics = false;
        }
    }

    if (regularPhysics) {
        controller->wantState = RE::hkpCharacterStateType::kInAir;
        controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kUnsupported;
    }




    spdlog::info("=== BALLISTIC MODE: ENTER (launch) === velocity ({:.1f}, {:.1f}, {:.1f}), speed: {:.1f}, gravity: {:.1f}, hadInitialSupport: {}, ghostMode: {}",
        velocity.x, velocity.y, velocity.z, speed, m_gravity, m_hadInitialSupport, m_ghostModeActive);
}

void BallisticController::StartFall(float gravity)
{

    bool isBeast = ClimbManager::IsPlayerInBeastForm();

    if (isBeast)
    {
        if (Config::options.regularPhysicsOnFallBeast)
        {
            regularPhysics = true;
        }
        else
        {
            regularPhysics = false;
        }
    }
    else
    {
        if (Config::options.regularPhysicsOnFall)
        {
            regularPhysics = true;
        }
        else
        {
            regularPhysics = false;
        }
    }


    if (m_inFlight) {
        spdlog::warn("BallisticController: Already in flight, ignoring fall start");
        return;
    }

    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        return;
    }

    // Store the gravity to restore on landing
    m_savedGravity = gravity;



    if (regularPhysics) {
        m_gravity = gravity * 700.0f;
        controller->gravity = gravity;
    }
    else
    {
        // Keep gravity disabled - we'll handle it ourselves
        controller->gravity = 0.0f;

        // Convert game's gravity to usable units/s²
        m_gravity = gravity * 700.0f;
    }











    // Start with zero velocity - just falling
    m_velocity = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    m_flightTime = 0.0f;
    m_inFlight = true;

    m_autoCatchResult = AutoCatchHand::kNone;













    // Initialize clearance tracking - check if we're starting with ground contact
    m_hadInitialSupport = (controller->supportBody.get() != nullptr);
    m_hasCleared = !m_hadInitialSupport;  // If no initial support, we're already "cleared"
    m_clearanceTimer = 0.0f;

    // Notify CriticalStrikeManager
    CriticalStrikeManager::GetSingleton()->OnLaunchStart();

    // No ghost mode for falls - only launches get ghost mode
    m_ghostModeActive = false;
    m_ghostModeTimer = 0.0f;
    m_ghostModeDuration = 0.0f;

    spdlog::info("=== BALLISTIC MODE: ENTER (fall) === gravity: {:.1f}, hadInitialSupport: {}",
        m_gravity, m_hadInitialSupport);
}

bool BallisticController::Update(float deltaTime)
{
    if (!m_inFlight) {
        return false;
    }

    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        Abort();
        return false;
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        Abort();
        return false;
    }

    m_flightTime += deltaTime;
    s_frameCount++;

    // Update clearance tracking
    m_clearanceTimer += deltaTime;
    if (!m_hasCleared && controller->supportBody.get() == nullptr) {
        m_hasCleared = true;
        spdlog::info("BallisticController: Cleared initial support after {:.3f}s", m_clearanceTimer);
    }

    // Update ghost mode timer
    if (m_ghostModeActive) {
        m_ghostModeTimer -= deltaTime;
        if (m_ghostModeTimer <= 0.0f) {
            // Ghost mode duration expired - restore collision
            SetPlayerWorldCollision(true);
            m_ghostModeActive = false;
            spdlog::info("BallisticController: Ghost mode EXPIRED after {:.3f}s", m_ghostModeDuration);
        }

        // Floor penetration check during ghost mode - prevent falling through world
        // Cast ray from above player down to find ground, limit how far below we can go
        constexpr float FLOOR_CHECK_HEIGHT = 80.0f;   // Start ray this high above player origin
        constexpr float FLOOR_CHECK_DISTANCE = 200.0f; // Check this far down (larger than climbing)
        constexpr float MAX_FLOOR_PENETRATION = 50.0f; // Max units player can be below ground surface

        RE::NiPoint3 playerPos = player->GetPosition();
        RE::NiPoint3 floorCheckOrigin = playerPos;
        floorCheckOrigin.z += FLOOR_CHECK_HEIGHT;
        RE::NiPoint3 downDir = { 0.0f, 0.0f, -1.0f };

        RaycastResult floorCheck = Raycast::CastRay(floorCheckOrigin, downDir, FLOOR_CHECK_DISTANCE);
        if (floorCheck.hit && IsGroundLayer(floorCheck.collisionLayer)) {
            // Ground surface is at: floorCheckOrigin.z - floorCheck.distance
            float groundZ = floorCheckOrigin.z - floorCheck.distance;
            float penetrationDepth = groundZ - playerPos.z;

            if (penetrationDepth > MAX_FLOOR_PENETRATION) {
                // Too far below ground - push player back up to max penetration depth
                RE::NiPoint3 correctedPos = playerPos;
                correctedPos.z = groundZ - MAX_FLOOR_PENETRATION;
                player->SetPosition(correctedPos, true);
                spdlog::info("BallisticController: Ghost mode floor correction - was {:.1f} below ground, corrected to {:.1f}",
                    penetrationDepth, MAX_FLOOR_PENETRATION);
            }
        }
    }

    // Update CriticalStrikeManager for enemy detection
    CriticalStrikeManager::GetSingleton()->Update(s_frameCount);

    // Track safe positions for exit correction fallback (every 50 frames)
    ClimbExitCorrector::GetSingleton()->UpdateSafePositionCheck();

    // Read current velocity from controller
    float havokScale = RE::bhkWorld::GetWorldScale();
    RE::hkVector4 hkVelocity;
    controller->GetLinearVelocityImpl(hkVelocity);

    if (!regularPhysics) {
        // Convert to game units for calculations
        RE::NiPoint3 velocity{


            hkVelocity.quad.m128_f32[0] / havokScale,
            hkVelocity.quad.m128_f32[1] / havokScale,
            hkVelocity.quad.m128_f32[2] / havokScale
        };

        // Calculate velocity delta for this frame
        float velocityDeltaZ = 0.0f;

        // Apply gravity as a delta
        velocityDeltaZ -= m_gravity * deltaTime;

        // Apply velocity delta (only modify Z, let game handle X/Y naturally)
        hkVelocity.quad.m128_f32[2] += velocityDeltaZ * havokScale;
        controller->SetLinearVelocityImpl(hkVelocity);

        // Update our cached velocity for logging/predictions
        m_velocity = velocity;
        m_velocity.z += velocityDeltaZ;
    }

    // Check for exit correction when speed drops below threshold or falling
    if (m_needsExitCorrection) {
        float currentSpeed = std::sqrt(m_velocity.x * m_velocity.x + m_velocity.y * m_velocity.y + m_velocity.z * m_velocity.z);
        bool isFalling = m_velocity.z < 0.0f;

        bool shouldCorrect = (currentSpeed < Config::options.launchExitCorrectionSpeedThreshold) || isFalling;

        // Also force correction if penetration is too deep
        if (!shouldCorrect && Config::options.exitCorrectionMaxPenetration > 0.0f) {
            auto* hmd = VRNodes::GetHMD();
            if (hmd) {
                RE::NiPoint3 hmdPos = hmd->world.translate;
                RE::NiPoint3 downDir = { 0.0f, 0.0f, -1.0f };
                constexpr float RAY_DISTANCE = 200.0f;

                RaycastResult groundCheck = Raycast::CastRay(hmdPos, downDir, RAY_DISTANCE);
                if (groundCheck.hit) {
                    constexpr float HMD_TO_FEET = 120.0f;
                    float feetZ = hmdPos.z - HMD_TO_FEET;
                    float groundZ = groundCheck.hitPoint.z;
                    float penetration = groundZ - feetZ;

                    if (penetration > Config::options.exitCorrectionMaxPenetration) {
                        spdlog::info("BallisticController: Penetration {:.1f} > max {:.1f}, forcing correction",
                            penetration, Config::options.exitCorrectionMaxPenetration);
                        shouldCorrect = true;
                    }
                }
            }
        }

        if (shouldCorrect) {
            spdlog::info("BallisticController: Starting exit correction, speed: {:.1f}, falling: {}", currentSpeed, isFalling);
            ClimbExitCorrector::GetSingleton()->StartCorrection(m_launchVelocity);
            m_needsExitCorrection = false;
        }
    }
    if (!regularPhysics) {
        // Always use kInAir - allows passing through small obstacles
        // Landing is detected via raycasting in CheckLanding()
        controller->wantState = RE::hkpCharacterStateType::kInAir;
        controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kUnsupported;
    }

    // Check for landing (after minimum flight time and below max landing velocity)
    float currentSpeed = std::sqrt(m_velocity.x * m_velocity.x + m_velocity.y * m_velocity.y + m_velocity.z * m_velocity.z);
    bool velocityAllowsLanding = (Config::options.maxLandingVelocity <= 0.0f) ||
        (currentSpeed <= Config::options.maxLandingVelocity);

    if (m_flightTime >= Config::options.minFlightTime && velocityAllowsLanding && CheckLanding()) {
        spdlog::info("=== BALLISTIC MODE: EXIT (landed) === flight time: {:.2f}s, landing speed: {:.1f}", m_flightTime, currentSpeed);

        // Re-enable collision (ghost mode off)
        SetPlayerWorldCollision(true);
        m_ghostModeActive = false;

        // Restore gravity BEFORE notifying CriticalStrikeManager
        // This ensures character state is normal when slow-mo ends
        controller->gravity = m_savedGravity;

        if (!regularPhysics) {
            // Reset character state to grounded (fixes stuck in kJumping)

            controller->wantState = RE::hkpCharacterStateType::kOnGround;
            controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kSupported;

            // Zero out any residual velocity to prevent drift
            RE::hkVector4 zero;
            zero.quad.m128_f32[0] = 0.0f;
            zero.quad.m128_f32[1] = 0.0f;
            zero.quad.m128_f32[2] = 0.0f;
            zero.quad.m128_f32[3] = 0.0f;
            controller->SetLinearVelocityImpl(zero);
        }



        m_inFlight = false;
        m_flightEndTime = std::chrono::steady_clock::now();
        m_hadFlight = true;

        // Note: Exit correction now runs mid-flight when speed drops below threshold
        // If it hasn't run yet (very fast landing), clear the flag anyway
        m_needsExitCorrection = false;

        // Notify CriticalStrikeManager AFTER state is fully reset
        CriticalStrikeManager::GetSingleton()->OnLaunchEnd();

        return false;  // No longer in flight
    }

    // Check for auto-catch opportunity (after minimum time, only while descending)
    if (m_flightTime >= AUTO_CATCH_MIN_TIME && m_velocity.z < 0.0f) {
        AutoCatchHand catchResult = CheckAutoCatch();
        if (catchResult != AutoCatchHand::kNone) {
            float speed = std::sqrt(m_velocity.x * m_velocity.x + m_velocity.y * m_velocity.y + m_velocity.z * m_velocity.z);
            spdlog::info("=== BALLISTIC MODE: EXIT (auto-catch {}) === flight time: {:.2f}s, speed: {:.1f}",
                (catchResult == AutoCatchHand::kLeft) ? "left" :
                (catchResult == AutoCatchHand::kRight) ? "right" : "both",
                m_flightTime, speed);

            // Store the result for ClimbManager to handle
            m_autoCatchResult = catchResult;

            // Re-enable collision (ghost mode off)
            SetPlayerWorldCollision(true);
            m_ghostModeActive = false;

            // Restore gravity and reset character state
            controller->gravity = m_savedGravity;
            controller->wantState = RE::hkpCharacterStateType::kOnGround;
            controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kSupported;

            // Zero out velocity
            RE::hkVector4 zero;
            zero.quad.m128_f32[0] = 0.0f;
            zero.quad.m128_f32[1] = 0.0f;
            zero.quad.m128_f32[2] = 0.0f;
            zero.quad.m128_f32[3] = 0.0f;
            controller->SetLinearVelocityImpl(zero);

            m_inFlight = false;
            m_flightEndTime = std::chrono::steady_clock::now();
            m_hadFlight = true;

            // Notify CriticalStrikeManager AFTER state is reset
            CriticalStrikeManager::GetSingleton()->OnLaunchEnd();

            return false;  // Flight ended by auto-catch
        }
    }

    return true;  // Still in flight
}

bool BallisticController::CheckLanding() const
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return true;  // Abort if no player
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        return true;  // Abort if no controller
    }

    // Check for collision via character controller's built-in detection
    bool hasSupport = (controller->supportBody.get() != nullptr);
    bool hasBumped = (controller->bumpedBody.get() != nullptr);
    bool hasCollision = hasSupport || hasBumped;

    if (!hasCollision) {
        return false;  // No collision detected - still in flight
    }

    // Collision detected - but should we ignore it?
    // If we started with ground contact, we need to either:
    // 1. Clear it first (supportBody becomes nullptr), OR
    // 2. Wait for the clearance timeout (couldn't get airborne)
    if (m_hadInitialSupport && !m_hasCleared) {
        // We haven't cleared yet - check if timeout elapsed
        if (m_clearanceTimer < CLEARANCE_TIMEOUT) {
            // Still in grace period - ignore collision to allow clearing obstacles
            return false;
        }
        // Timeout elapsed and we never cleared - we're stuck, land now
        spdlog::info("BallisticController: Clearance timeout ({:.2f}s) - never got airborne, landing",
            m_clearanceTimer);
        return true;
    }

    // Either we had no initial support, or we successfully cleared it
    // Any collision now means we've landed
    spdlog::trace("BallisticController: Landing detected (support: {}, bumped: {})",
        hasSupport, hasBumped);
    return true;
}

void BallisticController::Abort()
{
    if (!m_inFlight) {
        return;
    }

    // Re-enable collision (ghost mode off)
    SetPlayerWorldCollision(true);
    m_ghostModeActive = false;

    auto* player = RE::PlayerCharacter::GetSingleton();
    if (player) {
        auto* controller = player->GetCharController();
        if (controller) {
            // Restore gravity and reset character state
            controller->gravity = m_savedGravity;
            controller->wantState = RE::hkpCharacterStateType::kOnGround;
            controller->surfaceInfo.supportedState = RE::hkpSurfaceInfo::SupportedState::kSupported;
        }
    }

    m_inFlight = false;
    m_flightEndTime = std::chrono::steady_clock::now();
    m_hadFlight = true;

    // Notify CriticalStrikeManager AFTER state is reset
    CriticalStrikeManager::GetSingleton()->OnLaunchEnd();

    spdlog::info("=== BALLISTIC MODE: EXIT (aborted) === flight time: {:.2f}s", m_flightTime);
}

RE::NiPoint3 BallisticController::PredictLandingPosition() const
{
    if (!m_inFlight) {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (player) {
            return player->GetPosition();
        }
        return RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    }

    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    }

    RE::NiPoint3 pos = player->GetPosition();
    RE::NiPoint3 vel = m_velocity;

    // Simple prediction: simulate until Z velocity goes negative and we're below start
    // This is a rough estimate - could be improved with actual ground raycasting
    float t = 0.0f;
    constexpr float dt = 0.05f;  // 50ms steps
    constexpr float maxTime = 10.0f;  // Max prediction time

    while (t < maxTime) {
        vel.z -= m_gravity * dt;
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        pos.z += vel.z * dt;
        t += dt;

        // Simple ground check at predicted position
        if (vel.z < 0.0f) {
            RE::NiPoint3 down{ 0.0f, 0.0f, -1.0f };
            RaycastResult result = Raycast::CastRay(pos, down, 50.0f);
            if (result.hit && result.distance < 20.0f) {
                pos.z = result.hitPoint.z;
                return pos;
            }
        }
    }

    return pos;
}

bool BallisticController::IsInAutoCatchWindow() const
{
    // In flight = autocatch is active
    if (m_inFlight) {
        return true;
    }

    // No flight has occurred yet
    if (!m_hadFlight) {
        return false;
    }

    // Check if we're within the post-flight grace period
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - m_flightEndTime).count();
    return elapsed < POST_FLIGHT_AUTOCATCH_DURATION;
}

BallisticController::AutoCatchHand BallisticController::CheckAutoCatch() const
{
    uint8_t result = AutoCatchHand::kNone;

    // Beast forms (werewolf/vampire lord) can grab in any direction
    // Normal players only check downward
    if (ClimbManager::IsPlayerInBeastForm()) {
        // Use full multi-directional surface detection for beast forms
        if (ClimbSurfaceDetector::CanGrabSurface(true)) {
            result |= AutoCatchHand::kLeft;
        }
        if (ClimbSurfaceDetector::CanGrabSurface(false)) {
            result |= AutoCatchHand::kRight;
        }

    }
    else {
        // Normal players: cast rays world-down only
        static const RE::NiPoint3 worldDown{ 0.0f, 0.0f, -1.0f };

        if (ClimbSurfaceDetector::CastRayInDirection(true, worldDown)) {
            result |= AutoCatchHand::kLeft;
        }
        if (ClimbSurfaceDetector::CastRayInDirection(false, worldDown)) {
            result |= AutoCatchHand::kRight;
        }
    }

    return static_cast<AutoCatchHand>(result);
}

void BallisticController::SetPlayerWorldCollision(bool enabled)
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    auto* controller = player->GetCharController();
    if (!controller) {
        return;
    }

    // Cast to bhkCharProxyController to access GetCharacterProxy()
    auto* proxyController = skyrim_cast<RE::bhkCharProxyController*>(controller);
    if (!proxyController) {
        spdlog::warn("BallisticController: Character controller is not a proxy controller");
        return;
    }

    auto* charProxy = proxyController->GetCharacterProxy();
    if (!charProxy) {
        spdlog::warn("BallisticController: No character proxy found");
        return;
    }

    auto* phantom = charProxy->shapePhantom;
    if (!phantom) {
        spdlog::warn("BallisticController: No shape phantom found");
        return;
    }

    // Access the collidable's broadPhaseHandle to modify collision filter info
    // hkpShapePhantom inherits from hkpPhantom -> hkpWorldObject which has collidable at offset 0x20
    auto* collidable = phantom->GetCollidableRW();
    if (!collidable) {
        spdlog::warn("BallisticController: No collidable found on phantom");
        return;
    }

    // Log current state BEFORE any changes
    std::uint32_t currentFilterInfo = collidable->broadPhaseHandle.collisionFilterInfo;
    RE::COL_LAYER currentLayer = static_cast<RE::COL_LAYER>(currentFilterInfo & 0x7F);
    spdlog::info("BallisticController: SetPlayerWorldCollision({}) - current layer: {}, filterInfo: 0x{:08X}, m_collisionDisabled: {}",
        enabled, static_cast<int>(currentLayer), currentFilterInfo, m_collisionDisabled);

    if (enabled) {
        // Restore original collision layer
        if (m_collisionDisabled && m_originalFilterInfo != 0) {
            spdlog::info("BallisticController: Restoring filterInfo from 0x{:08X} to 0x{:08X}",
                currentFilterInfo, m_originalFilterInfo);
            collidable->broadPhaseHandle.collisionFilterInfo = m_originalFilterInfo;
            m_collisionDisabled = false;

            // Verify the write took effect
            std::uint32_t verifyFilterInfo = collidable->broadPhaseHandle.collisionFilterInfo;
            RE::COL_LAYER verifyLayer = static_cast<RE::COL_LAYER>(verifyFilterInfo & 0x7F);
            spdlog::info("BallisticController: Collision layer RESTORED to {} (filterInfo: 0x{:08X}, verify: 0x{:08X})",
                static_cast<int>(verifyLayer), m_originalFilterInfo, verifyFilterInfo);

        }
        else {
            spdlog::warn("BallisticController: Cannot restore - m_collisionDisabled: {}, m_originalFilterInfo: 0x{:08X}",
                m_collisionDisabled, m_originalFilterInfo);
        }

    }
    else {
        // Set to NonCollidable layer
        if (!m_collisionDisabled) {
            // Store original
            m_originalFilterInfo = collidable->broadPhaseHandle.collisionFilterInfo;
            RE::COL_LAYER originalLayer = static_cast<RE::COL_LAYER>(m_originalFilterInfo & 0x7F);

            // Clear layer bits (0x7F) and set to kNonCollidable
            std::uint32_t newFilterInfo = (m_originalFilterInfo & ~0x7Fu) |
                static_cast<std::uint32_t>(RE::COL_LAYER::kNonCollidable);
            collidable->broadPhaseHandle.collisionFilterInfo = newFilterInfo;
            m_collisionDisabled = true;

            // Verify the write took effect
            std::uint32_t verifyFilterInfo = collidable->broadPhaseHandle.collisionFilterInfo;
            spdlog::info("BallisticController: Ghost mode ON - layer {} -> 15, filterInfo: 0x{:08X} -> 0x{:08X} (verify: 0x{:08X})",
                static_cast<int>(originalLayer), m_originalFilterInfo, newFilterInfo, verifyFilterInfo);

        }
        else {
            spdlog::warn("BallisticController: Already in ghost mode, skipping disable");
        }
    }
}

bool BallisticController::ShouldEnterGhostMode(const RE::NiPoint3& velocity, float speed) const
{
    // Check minimum speed requirement
    if (speed < Config::options.ghostModeMinSpeed) {
        spdlog::trace("BallisticController: Ghost mode skipped - speed {:.1f} < min {:.1f}",
            speed, Config::options.ghostModeMinSpeed);
        return false;
    }

    // Check if ghost mode is enabled (duration > 0)
    float duration = Config::options.ghostModeDuration;
    if (duration <= 0.0f) {
        spdlog::trace("BallisticController: Ghost mode disabled (duration = 0)");
        return false;
    }

    // Get HMD position as ray origin
    auto* hmd = VRNodes::GetHMD();
    if (!hmd) {
        spdlog::warn("BallisticController: No HMD node for ghost mode ray check");
        return false;
    }
    RE::NiPoint3 origin = hmd->world.translate;

    // Calculate ray direction (normalized velocity)
    RE::NiPoint3 direction{
        velocity.x / speed,
        velocity.y / speed,
        velocity.z / speed
    };

    // Calculate ray length = distance traveled during ghost mode duration + player size safety margin
    constexpr float PLAYER_SIZE_MARGIN = 120.0f;  // Account for player collision capsule
    float travelDistance = speed * duration;
    float rayLength = travelDistance + PLAYER_SIZE_MARGIN;

    // Cast the ray to check for obstacles
    RaycastResult result = Raycast::CastRay(origin, direction, rayLength);

    if (result.hit) {
        spdlog::info("BallisticController: Ghost mode DENIED - ray hit {} at distance {:.1f} (layer {})",
            result.hitPoint.x, result.distance, static_cast<int>(result.collisionLayer));
        return false;
    }

    spdlog::info("BallisticController: Ghost mode APPROVED - clear path for {:.1f} units (travel: {:.1f} + margin: {:.1f})",
        rayLength, travelDistance, PLAYER_SIZE_MARGIN);
    return true;
}
