#pragma once

#include <map>
#include <string>

namespace Config {
    struct Options {
        // ===== Climbing (basic on/off and physics) =====
        bool climbingEnabled = true;                 // Enable climbing for player (beast forms always can)
        bool latchHapticsEnabled = true;             // Haptic pulse on successful latch
        float latchHapticDuration = 14.4f;           // Haptic pulse duration (SkyrimVR internal units)
        float minLaunchSpeed = 5.0f;                 // Minimum speed to trigger launch (units/s)
        float horizontalLaunchBoost = 1.0f;          // Extra multiplier for horizontal movement
        float velocityHistoryTime = 0.1f;            // Seconds of velocity samples to keep
        int maxVelocitySamples = 10;                 // Max velocity samples to track
        float velocityWeightExponent = 1.0f;         // Exponent for velocity weighting (1.0=linear, 2.0=quadratic)
        float launchDirectionFilter = 90.0f;         // Max angle deviation from dominant direction (degrees, 0=disabled)
        float ghostModeDuration = 0.15f;             // Duration of ghost mode after launch (seconds, 0=disabled)
        float ghostModeMinSpeed = 200.0f;            // Minimum launch speed to trigger ghost mode (units/s)
        float minFlightTime = 0.5f;                  // Minimum time before allowing landing (seconds)
        float maxLandingVelocity = 200.0f;           // Max velocity to allow landing (units/s, 0=disabled)

        // ===== Climbing Ability (base values for naked, non-encumbered player) =====
        float smoothingSpeed = 13.0f;                // Exponential smoothing factor for movement
        float baseLaunchSpeed = 600.0f;              // Base max launch speed (units/s)
        float maxLaunchMultiplier = 1.25f;           // Max velocity multiplier for launches
        float baseStaminaCost = 1.0f;                // Base stamina drain per second
        float grabRayLength = 6.75f;                 // Ray length for surface detection (game units)

        // ===== Climbing Ability Per Worn Weight =====
        bool weightScalingEnabled = true;            // Enable weight-based ability scaling
        float maxArmorWeight = 80.0f;                // Weight considered "max" for interpolation
        float minLaunchSpeedWeighted = 350.0f;       // Min launch speed at max armor weight
        float minLaunchMultiplier = 0.9f;            // Min launch multiplier at max armor weight
        float maxStaminaCost = 16.0f;                // Max stamina cost at max armor weight

        // ===== Overencumbered =====
        bool overencumberedEnabled = true;           // Enable overencumbered penalties
        float overencumberedMaxLaunchSpeed = 200.0f; // Max launch speed when overencumbered
        float overencumberedStaminaCost = 20.0f;     // Stamina cost when overencumbered

        // ===== Beast Form Bonuses =====
        float beastMaxLaunchSpeed = 880.0f;          // Higher launch cap for werewolf/vampire lord
        float beastLaunchMultiplier = 1.25f;         // Stronger launches for beast forms
        float beastGrabRayLength = 6.75f;            // Extended grab reach for beast forms
        float khajiitMaxLaunchSpeedBonus = 100.0f;   // Bonus launch speed for Khajiit
        float argonianMaxLaunchSpeedBonus = 50.0f;   // Bonus launch speed for Argonian

        // ===== Climbing Damage =====
        bool climbingDamageEnabled = true;           // Enable grip release on damage
        float damageThresholdPercent = 7.0f;         // % of max health to trigger release

        // ===== Critical Strike =====
        bool criticalStrikeEnabled = true;           // Enable critical strike system
        bool criticalAngleCheckEnabled = true;       // Require HMD to face movement direction
        int criticalCheckInterval = 3;               // Frames between critical strike checks
        float criticalMinSpeed = 1.0f;               // Minimum player speed to trigger (game units/s)
        float criticalMinDiveAngle = 30.0f;          // Minimum angle below horizontal (degrees, 0=horizontal, 90=straight down)
        float criticalRayDistance = 270.0f;          // Ray distance to check for impact point (game units)
        float criticalDetectionRadius = 80.0f;       // Radius around impact point to detect actors (game units)
        float criticalHmdAlignmentAngle = 140.0f;    // Max angle between HMD forward and movement
        bool criticalHostilesOnly = false;           // Only trigger on hostile actors (false = any actor)
        bool criticalEndOnLand = true;               // End slow-mo when landing (unless target was hit)
        float slowdownDuration = 4.0f;               // How long slow-mo lasts (seconds)
        float worldSlowdown = 0.15f;                 // World time multiplier (0.15 = 15% speed)
        float playerSlowdown = 0.15f;                // Player time multiplier during slow-mo
        float ragdollMagnitude = 5.0f;               // Force applied when ragdolling target
        float ragdollRadius = 300.0f;                // Radius around impact point for ragdoll eligibility
        bool ragdollOnHit = true;                    // Enable ragdoll on hit during slow-mo
        bool disableNPCCollision = true;             // Disable collision with NPCs during slow-mo flight
        float postLandDuration = 1.2f;               // Extra slow-mo time after landing (seconds)
        float postHitDuration = 1.5f;                // Extra slow-mo time after hitting target (seconds)

        //Aelove's tweaks
        float minStamina = 75.0f;
        bool enableFallDamage = true;

        // ===== Launching (ballistic flight after releasing grip) =====
        float launchExitCorrectionSpeedThreshold = 150.0f; // Speed below which exit correction triggers

        // ===== Exit Correction (smooth position adjustment after launch) =====
        float exitCorrectionMaxPenetration = 60.0f;  // Max units below ground before forcing immediate correction
        float exitCorrectionSecondsPerUnit = 0.003f; // Duration per unit of distance (seconds/unit)
        float exitCorrectionControlPointScale = 0.4f;// How much velocity influences curve shape

        // ===== Sound =====
        bool soundEnabled = true;                    // Enable climbing/launch sounds
        float soundVolume = 0.5f;                    // Sound volume (0-1), scaled by game master volume

        // ===== Debug / Development =====
        bool hotReloadEnabled = false;                // Hot reload INI when modified (disable for release)
    };

    extern Options options;

    // Type maps for runtime lookup (console commands, etc.)
    extern std::map<std::string, float*, std::less<>> floatMap;
    extern std::map<std::string, int*, std::less<>> intMap;
    extern std::map<std::string, bool*, std::less<>> boolMap;

    // Read all config from INI file (creates default if not found)
    bool ReadConfigOptions();

    // Reload config if file was modified (call periodically for hot-reload)
    bool ReloadIfModified();

    // Runtime setting access by name (for console commands)
    bool SetSettingDouble(const std::string_view& name, double val);
    bool GetSettingDouble(const std::string_view& name, double& out);

    // Get path to INI file
    const std::string& GetConfigPath();
}
