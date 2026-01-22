#pragma once

#include "RE/Skyrim.h"
#include <chrono>

// Controls player trajectory after launch until touchdown
// Takes over from the game's physics to ensure smooth ballistic flight
// Can be extended later for aiming/trajectory prediction
class BallisticController
{
public:
    static BallisticController* GetSingleton();

    // Start ballistic flight with initial velocity
    // Takes full control of player movement until landing
    // gravity: the game's gravity value to use (pass the saved value, not current which may be 0)
    void Launch(const RE::NiPoint3& velocity, float gravity);

    // Start a controlled fall (no initial velocity, just gravity)
    // Used when player releases grip without enough momentum to launch
    // Still provides auto-catch protection while falling
    void StartFall(float gravity);

    // Call every physics frame to update trajectory
    // Returns true if still in flight, false if landed
    bool Update(float deltaTime);

    // Check if currently controlling the player
    bool IsInFlight() const { return m_inFlight; }

    // Abort flight and return control to game
    void Abort();

    // Get current velocity (useful for aiming system later)
    RE::NiPoint3 GetVelocity() const { return m_velocity; }

    // Get predicted landing position (for aiming system)
    RE::NiPoint3 PredictLandingPosition() const;

    // Get current gravity being used
    float GetGravity() const { return m_gravity; }

    // Auto-catch: Check if player can "catch" themselves mid-flight
    // Returns bitmask: 0 = no catch, 1 = left hand, 2 = right hand, 3 = both
    enum AutoCatchHand : uint8_t {
        kNone = 0,
        kLeft = 1,
        kRight = 2,
        kBoth = 3
    };

    // Get which hand(s) triggered an auto-catch this frame (valid after Update returns false)
    AutoCatchHand GetAutoCatchResult() const { return m_autoCatchResult; }

    // Clear the auto-catch result (call after handling it)
    void ClearAutoCatchResult() { m_autoCatchResult = AutoCatchHand::kNone; }

    // Request position correction on landing (called by ClimbManager on climb exit)
    void RequestExitCorrection() { m_needsExitCorrection = true; }

    // Cancel exit correction request (when player re-grabs)
    void CancelExitCorrection() { m_needsExitCorrection = false; }

    // Configuration
    static constexpr float GROUND_CHECK_DISTANCE = 10.0f;  // Raycast distance for ground detection (fallback)
    static constexpr float SLOPE_CHECK_DISTANCE = 40.0f;   // Raycast distance for slope detection (velocity-directed)
    static constexpr float WALL_CHECK_DISTANCE = 30.0f;    // Raycast distance for wall detection in velocity direction
    // MIN_FLIGHT_TIME moved to Config::options.minFlightTime (INI configurable)
    static constexpr float AUTO_CATCH_MIN_TIME = 0.05f; // Minimum flight time before auto-catch can trigger
    static constexpr float POST_FLIGHT_AUTOCATCH_DURATION = 5.0f; // Keep autocatch active this long after flight ends
    static constexpr float CLEARANCE_TIMEOUT = 0.25f;  // Max time to wait for clearing initial support before landing
    static constexpr float ESCAPE_ACCELERATION = 8000.0f;  // Upward acceleration when feet inside ground (units/s²)

    // Check if we're in the post-flight autocatch window
    // Returns true if flight ended within POST_FLIGHT_AUTOCATCH_DURATION seconds ago
    bool IsInAutoCatchWindow() const;

    // Check if hands are near a surface they could catch
    // For beast forms: casts rays in all directions
    // For normal: casts rays world-down only
    // Returns which hand(s) detected a catchable surface
    AutoCatchHand CheckAutoCatch() const;

private:
    BallisticController() = default;
    ~BallisticController() = default;
    BallisticController(const BallisticController&) = delete;
    BallisticController& operator=(const BallisticController&) = delete;

    // Check if player has landed (collision detected)
    bool CheckLanding() const;

    // State
    bool m_inFlight = false;
    bool m_needsExitCorrection = false;  // True if ClimbExitCorrector should run on landing
    AutoCatchHand m_autoCatchResult = AutoCatchHand::kNone;
    RE::NiPoint3 m_velocity{ 0.0f, 0.0f, 0.0f };  // Cached velocity (read from controller each frame)
    RE::NiPoint3 m_launchVelocity{ 0.0f, 0.0f, 0.0f };  // Original launch velocity (for exit correction)
    float m_flightTime = 0.0f;  // Time since launch
    float m_gravity = 0.0f;     // Gravity to apply as delta (units/s²)
    bool regularPhysics = false; //AELOVE : Check if we're using the regular Havok physics or the mod's version

    // Track when flight ended for post-flight autocatch window
    std::chrono::steady_clock::time_point m_flightEndTime;
    bool m_hadFlight = false;  // True after first flight (to know if m_flightEndTime is valid)

    // Saved state to restore after landing
    float m_savedGravity = 0.0f;

    // Clearance tracking - allows launching through initial contact
    bool m_hadInitialSupport = false;  // True if supportBody existed at launch
    bool m_hasCleared = false;         // True once supportBody became nullptr
    float m_clearanceTimer = 0.0f;     // Time since launch (for clearance timeout)

    // Ghost mode - temporarily disables player collision after launch
    bool m_collisionDisabled = false;        // True if we disabled collision (to know to restore it)
    std::uint32_t m_originalFilterInfo = 0;  // Original collision filter info to restore
    bool m_ghostModeActive = false;          // True if ghost mode is currently active
    float m_ghostModeTimer = 0.0f;           // Time remaining in ghost mode
    float m_ghostModeDuration = 0.0f;        // Total ghost mode duration for this launch

    // Helper to enable/disable player world collision via collision layer
    void SetPlayerWorldCollision(bool enabled);

    // Check if ghost mode should be entered (ray check for clear path)
    bool ShouldEnterGhostMode(const RE::NiPoint3& velocity, float speed) const;
};
