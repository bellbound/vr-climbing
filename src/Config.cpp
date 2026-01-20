#include "Config.h"

#include <Windows.h>
#include <filesystem>
#include <fstream>
#include <spdlog/spdlog.h>

namespace Config {
    Options options;
    std::map<std::string, float*, std::less<>> floatMap;
    std::map<std::string, int*, std::less<>> intMap;
    std::map<std::string, bool*, std::less<>> boolMap;

    static bool g_registrationComplete = false;
    static std::string g_configPath;

    // ===== Default INI content =====
    static constexpr const char* DEFAULT_INI_CONTENT = R"(; VRClimbing Configuration
; Delete this file to regenerate with defaults

[Climbing]
; Enable climbing for player (1=enabled, 0=disabled). Beast forms can always climb.
enabled=1
; Enable haptic pulse when a hand successfully latches onto a surface (1=enabled, 0=disabled)
latchHapticsEnabled=1
; Haptic pulse duration/intensity (SkyrimVR internal units)
latchHapticDuration=14.4
; Minimum speed to trigger launch when releasing grip (units/sec)
minLaunchSpeed=5.0
; Extra multiplier for horizontal (X/Y) movement (1.0 = same as vertical)
horizontalLaunchBoost=1.0
; Seconds of velocity samples to keep for launch calculation
velocityHistoryTime=0.1
; Maximum number of velocity samples to track
maxVelocitySamples=10
; Exponent for velocity weighting in launch calculation
; 1.0 = linear (fast frames have proportional influence)
; 2.0 = quadratic (fast frames dominate more strongly)
velocityWeightExponent=1.0
; Max angle deviation from dominant direction (degrees)
; Samples outside this cone are rejected as outliers (e.g. wrist rotation)
; Set to 0 to disable filtering
launchDirectionFilter=90.0
; Ghost mode: temporarily disable world collision after launch
; Duration in seconds (0 = disabled)
ghostModeDuration=0.15
; Minimum launch speed to trigger ghost mode (units/sec)
; Ghost mode only activates if path is clear (ray check)
ghostModeMinSpeed=200.0
; Minimum time in air before landing is allowed (seconds)
; Prevents instant landing when launching from ground contact
minFlightTime=0.5
; Maximum velocity to allow landing (units/sec, 0=disabled)
; Player won't land while moving faster than this
maxLandingVelocity=200.0

[ClimbingAbility]
; Variables that affect how easy it is to climb
; Base values are for a naked player that is not overencumbered
; Exponential smoothing factor for movement (higher = snappier)
smoothingSpeed=13.0
; Base max launch speed (units/sec)
baseLaunchSpeed=600.0
; Max velocity multiplier applied to launches
maxLaunchMultiplier=1.25
; Base stamina drain per second while climbing (set to 0 to disable stamina drain)
baseStaminaCost=8.0
; Ray length for detecting climbable surfaces (game units)
grabRayLength=6.75

[ClimbingAbilityPerWornWeight]
; When enabled, climbing ability scales with armor weight
; We interpolate between max (naked) and min (fully armored) values
; 80 weight units is considered "max armor" - anything above is treated as 80
enabled=1
; Min launch speed at max armor weight
minLaunchSpeed=350.0
; Min launch multiplier at max armor weight
minLaunchMultiplier=0.9
; Max stamina cost per second at max armor weight
maxStaminaCost=16.0

[Overencumbered]
; Penalties when player is over-encumbered (inventory weight > carry weight)
; Overwrites other calculations (except beast form)
enabled=1
; Max launch speed when overencumbered
maxLaunchSpeed=200.0
; Stamina cost per second when overencumbered
staminaCost=20.0

[BeastForm]
; Bonuses for werewolf/vampire lord forms
; Beast forms ignore armor weight and cannot be over-encumbered for climbing
maxLaunchSpeed=880.0
launchMultiplier=1.25
grabRayLength=6.75
; Race-specific bonuses (added to calculated max launch speed)
khajiitMaxLaunchSpeedBonus=100.0
argonianMaxLaunchSpeedBonus=50.0

[ClimbingDamage]
; Enable grip release when taking damage (1=enabled, 0=disabled)
; Beast forms (werewolf/vampire lord) are immune to damage-based grip release
enabled=1
; Percentage of max health that triggers grip release (0-100)
damageThresholdPercent=7.0

[CriticalStrike]
; Enable critical strike system (1=enabled, 0=disabled)
enabled=1
; Require HMD to face movement direction (1=enabled, 0=disabled)
angleCheckEnabled=1
; Frames between critical strike checks during flight
checkInterval=3
; Minimum player speed to trigger critical strike (game units/sec)
minSpeed=1.0
; Minimum angle below horizontal for dive attack (degrees, 0=horizontal, 90=straight down)
minDiveAngle=30.0
; Ray distance to check for impact point in velocity direction (game units)
rayDistance=270.0
; Radius around impact point to detect actors (game units)
detectionRadius=80.0
; Radius around impact point in which your attacks will ragdoll on hit after landing
ragdollRadius=300.0
; Max angle between HMD forward and movement direction (degrees)
hmdAlignmentAngle=140.0
; Only trigger on hostile actors (0=any actor, 1=hostiles only)
hostilesOnly=0
; End slow-mo immediately when landing (unless target was hit) (1=enabled, 0=disabled)
endOnLand=1

; World time multiplier during slow-mo (0.15 = 15% speed)
worldSlowdown=0.15
; Player time multiplier during slow-mo
playerSlowdown=0.15
; Force applied when ragdolling target
ragdollMagnitude=5.0
; Enable ragdoll on hit during slow-mo (1=enabled, 0=disabled)
ragdollOnHit=1
; Disable collision with NPCs during slow-mo flight (1=enabled, 0=disabled)
disableNPCCollision=1
; Max. slow motion during flight / airtime
slowdownDuration=4.0
; Extra slow-mo time after landing without hitting target (seconds)
postLandDuration=1.2
; Extra slow-mo time after hitting target (seconds)
postHitDuration=1.5

[Launching]
; Speed threshold below which exit correction triggers (units/sec)
exitCorrectionSpeedThreshold=150.0

[ExitCorrection]
; Smooth position adjustment after launching (prevents clipping through geometry)
; Duration per unit of linear distance (seconds per game unit)
secondsPerUnit=0.003
; How much initial velocity influences the curve shape (0-1)
controlPointScale=0.4

[Sound]
; Enable climbing and launch sounds (1=enabled, 0=disabled)
enabled=1
; Sound volume (0.0-1.0) - multiplied by game's master volume
volume=0.5

[Debug]
; Hot reload INI when file is modified (1=enabled, 0=disabled)
; Disable this for release builds to avoid file system checks every frame
hotReloadEnabled=0

[AeloveTweaks]
; Sets a minimum amount of Stamina required to be able to climb
; Set to 0 to disable
minStamina=75
; Use the regular Havok physics when falling after climbing in human form
; If true, you can no longer launch yourself and you suffer from fall damage normally
regularPhysicsOnFall=true
; Use the regular Havok physics when falling after climbing in werewolf or vampire lord form
regularPhysicsOnFallBeast=false
; Base stamina drain per second while climbing in beast form (set to 0 to disable)
baseStaminaCostBeast=4.0
)";

    // ===== Low-level INI readers using Windows API =====
    static std::string GetConfigOption(const char* section, const char* key) {
        const std::string& configPath = GetConfigPath();
        if (configPath.empty()) return "";

        char buffer[256];
        GetPrivateProfileStringA(section, key, "", buffer, sizeof(buffer), configPath.c_str());
        return buffer;
    }

    static bool GetConfigOptionFloat(const char* section, const char* key, float* out) {
        std::string data = GetConfigOption(section, key);
        if (data.empty()) return false;
        try {
            *out = std::stof(data);
            return true;
        } catch (...) {
            spdlog::warn("Config: Failed to parse float for {}/{}", section, key);
            return false;
        }
    }

    static bool GetConfigOptionInt(const char* section, const char* key, int* out) {
        std::string data = GetConfigOption(section, key);
        if (data.empty()) return false;
        try {
            *out = std::stoi(data);
            return true;
        } catch (...) {
            spdlog::warn("Config: Failed to parse int for {}/{}", section, key);
            return false;
        }
    }

    static bool GetConfigOptionBool(const char* section, const char* key, bool* out) {
        std::string data = GetConfigOption(section, key);
        if (data.empty()) return false;
        try {
            int val = std::stoi(data);
            *out = (val != 0);
            return true;
        } catch (...) {
            spdlog::warn("Config: Failed to parse bool for {}/{}", section, key);
            return false;
        }
    }

    // ===== Register functions - add to map AND read from INI =====
    static bool RegisterFloat(const char* section, const std::string& name, float& val) {
        if (!g_registrationComplete) floatMap[name] = &val;
        if (!GetConfigOptionFloat(section, name.c_str(), &val)) {
            spdlog::debug("Config: {} not found in [{}], using default {}", name, section, val);
            return false;
        }
        spdlog::debug("Config: [{}] {} = {}", section, name, val);
        return true;
    }

    static bool RegisterInt(const char* section, const std::string& name, int& val) {
        if (!g_registrationComplete) intMap[name] = &val;
        if (!GetConfigOptionInt(section, name.c_str(), &val)) {
            spdlog::debug("Config: {} not found in [{}], using default {}", name, section, val);
            return false;
        }
        spdlog::debug("Config: [{}] {} = {}", section, name, val);
        return true;
    }

    static bool RegisterBool(const char* section, const std::string& name, bool& val) {
        if (!g_registrationComplete) boolMap[name] = &val;
        if (!GetConfigOptionBool(section, name.c_str(), &val)) {
            spdlog::debug("Config: {} not found in [{}], using default {}", name, section, val ? "true" : "false");
            return false;
        }
        spdlog::debug("Config: [{}] {} = {}", section, name, val ? "true" : "false");
        return true;
    }

    // ===== Create default INI file =====
    static bool CreateDefaultConfigFile(const std::string& path) {
        spdlog::info("Config: Creating default config file at {}", path);

        // Ensure directory exists
        std::filesystem::path filePath(path);
        std::filesystem::path dirPath = filePath.parent_path();

        try {
            if (!std::filesystem::exists(dirPath)) {
                std::filesystem::create_directories(dirPath);
                spdlog::info("Config: Created directory {}", dirPath.string());
            }
        } catch (const std::exception& e) {
            spdlog::error("Config: Failed to create directory: {}", e.what());
            return false;
        }

        // Write default content
        std::ofstream file(path);
        if (!file.is_open()) {
            spdlog::error("Config: Failed to create config file at {}", path);
            return false;
        }

        file << DEFAULT_INI_CONTENT;
        file.close();

        spdlog::info("Config: Default config file created successfully");
        return true;
    }

    bool ReadConfigOptions() {
        const std::string& path = GetConfigPath();

        // Check if file exists, create default if not
        if (!std::filesystem::exists(path)) {
            spdlog::info("Config: Config file not found, creating default");
            if (!CreateDefaultConfigFile(path)) {
                spdlog::error("Config: Failed to create default config, using built-in defaults");
            }
        }

        spdlog::info("Config: Reading config from {}", path);

        // Reset options to defaults before reading
        // This ensures any missing values use defaults
        options = Options{};

        // Climbing settings (basic physics)
        RegisterBool("Climbing", "enabled", options.climbingEnabled);
        RegisterBool("Climbing", "latchHapticsEnabled", options.latchHapticsEnabled);
        RegisterFloat("Climbing", "latchHapticDuration", options.latchHapticDuration);
        RegisterFloat("Climbing", "minLaunchSpeed", options.minLaunchSpeed);
        RegisterFloat("Climbing", "horizontalLaunchBoost", options.horizontalLaunchBoost);
        RegisterFloat("Climbing", "velocityHistoryTime", options.velocityHistoryTime);
        RegisterInt("Climbing", "maxVelocitySamples", options.maxVelocitySamples);
        RegisterFloat("Climbing", "velocityWeightExponent", options.velocityWeightExponent);
        RegisterFloat("Climbing", "launchDirectionFilter", options.launchDirectionFilter);
        RegisterFloat("Climbing", "ghostModeDuration", options.ghostModeDuration);
        RegisterFloat("Climbing", "ghostModeMinSpeed", options.ghostModeMinSpeed);
        RegisterFloat("Climbing", "minFlightTime", options.minFlightTime);
        RegisterFloat("Climbing", "maxLandingVelocity", options.maxLandingVelocity);

        // Climbing Ability (base values for naked player)
        RegisterFloat("ClimbingAbility", "smoothingSpeed", options.smoothingSpeed);
        RegisterFloat("ClimbingAbility", "baseLaunchSpeed", options.baseLaunchSpeed);
        RegisterFloat("ClimbingAbility", "maxLaunchMultiplier", options.maxLaunchMultiplier);
        RegisterFloat("ClimbingAbility", "baseStaminaCost", options.baseStaminaCost);
        RegisterFloat("ClimbingAbility", "grabRayLength", options.grabRayLength);

        // Climbing Ability Per Worn Weight
        RegisterBool("ClimbingAbilityPerWornWeight", "enabled", options.weightScalingEnabled);
        RegisterFloat("ClimbingAbilityPerWornWeight", "minLaunchSpeed", options.minLaunchSpeedWeighted);
        RegisterFloat("ClimbingAbilityPerWornWeight", "minLaunchMultiplier", options.minLaunchMultiplier);
        RegisterFloat("ClimbingAbilityPerWornWeight", "maxStaminaCost", options.maxStaminaCost);

        // Overencumbered
        RegisterBool("Overencumbered", "enabled", options.overencumberedEnabled);
        RegisterFloat("Overencumbered", "maxLaunchSpeed", options.overencumberedMaxLaunchSpeed);
        RegisterFloat("Overencumbered", "staminaCost", options.overencumberedStaminaCost);

        // Beast form settings
        RegisterFloat("BeastForm", "maxLaunchSpeed", options.beastMaxLaunchSpeed);
        RegisterFloat("BeastForm", "launchMultiplier", options.beastLaunchMultiplier);
        RegisterFloat("BeastForm", "grabRayLength", options.beastGrabRayLength);
        RegisterFloat("BeastForm", "khajiitMaxLaunchSpeedBonus", options.khajiitMaxLaunchSpeedBonus);
        RegisterFloat("BeastForm", "argonianMaxLaunchSpeedBonus", options.argonianMaxLaunchSpeedBonus);

        // Climbing Damage settings
        RegisterBool("ClimbingDamage", "enabled", options.climbingDamageEnabled);
        RegisterFloat("ClimbingDamage", "damageThresholdPercent", options.damageThresholdPercent);

        // Critical Strike settings
        RegisterBool("CriticalStrike", "enabled", options.criticalStrikeEnabled);
        RegisterBool("CriticalStrike", "angleCheckEnabled", options.criticalAngleCheckEnabled);
        RegisterInt("CriticalStrike", "checkInterval", options.criticalCheckInterval);
        RegisterFloat("CriticalStrike", "minSpeed", options.criticalMinSpeed);
        RegisterFloat("CriticalStrike", "minDiveAngle", options.criticalMinDiveAngle);
        RegisterFloat("CriticalStrike", "rayDistance", options.criticalRayDistance);
        RegisterFloat("CriticalStrike", "detectionRadius", options.criticalDetectionRadius);
        RegisterFloat("CriticalStrike", "hmdAlignmentAngle", options.criticalHmdAlignmentAngle);
        RegisterBool("CriticalStrike", "hostilesOnly", options.criticalHostilesOnly);
        RegisterBool("CriticalStrike", "endOnLand", options.criticalEndOnLand);
        RegisterFloat("CriticalStrike", "slowdownDuration", options.slowdownDuration);
        RegisterFloat("CriticalStrike", "worldSlowdown", options.worldSlowdown);
        RegisterFloat("CriticalStrike", "playerSlowdown", options.playerSlowdown);
        RegisterFloat("CriticalStrike", "ragdollMagnitude", options.ragdollMagnitude);
        RegisterFloat("CriticalStrike", "ragdollRadius", options.ragdollRadius);
        RegisterBool("CriticalStrike", "ragdollOnHit", options.ragdollOnHit);
        RegisterBool("CriticalStrike", "disableNPCCollision", options.disableNPCCollision);
        RegisterFloat("CriticalStrike", "postLandDuration", options.postLandDuration);
        RegisterFloat("CriticalStrike", "postHitDuration", options.postHitDuration);

        // Launching settings
        
        RegisterFloat("Launching", "exitCorrectionSpeedThreshold", options.launchExitCorrectionSpeedThreshold);
        // Exit Correction settings
        RegisterFloat("ExitCorrection", "maxPenetration", options.exitCorrectionMaxPenetration);
        RegisterFloat("ExitCorrection", "secondsPerUnit", options.exitCorrectionSecondsPerUnit);
        RegisterFloat("ExitCorrection", "controlPointScale", options.exitCorrectionControlPointScale);

        // Sound settings
        RegisterBool("Sound", "enabled", options.soundEnabled);
        RegisterFloat("Sound", "volume", options.soundVolume);

        // Debug settings
        RegisterBool("Debug", "hotReloadEnabled", options.hotReloadEnabled);

        // Aelove Tweaks
        RegisterFloat("AeloveTweaks", "minStamina", options.minStamina);
        RegisterBool("AeloveTweaks", "regularPhysicsOnFall", options.regularPhysicsOnFall);
        RegisterBool("AeloveTweaks", "regularPhysicsOnFallBeast", options.regularPhysicsOnFall);
        RegisterFloat("AeloveTweaks", "baseStaminaCostBeast", options.baseStaminaCostBeast);

        g_registrationComplete = true;

        spdlog::info("Config: Loaded successfully");
        return true;
    }

    bool ReloadIfModified() {
        namespace fs = std::filesystem;
        static long long lastModifiedTime = 0;

        const std::string& path = GetConfigPath();
        if (path.empty()) return false;

        try {
            if (!fs::exists(path)) return false;

            auto ftime = fs::last_write_time(path);
            auto time = ftime.time_since_epoch().count();
            if (time > lastModifiedTime) {
                lastModifiedTime = time;

                // Skip first call (initial load already handled)
                static bool firstCall = true;
                if (firstCall) {
                    firstCall = false;
                    return false;
                }

                spdlog::info("Config: File modified, reloading...");
                // Flush Windows INI cache - GetPrivateProfileString caches file contents
                WritePrivateProfileStringA(NULL, NULL, NULL, path.c_str());
                ReadConfigOptions();
                return true;
            }
        } catch (const std::exception& e) {
            spdlog::warn("Config: Error checking file modification: {}", e.what());
        }
        return false;
    }

    const std::string& GetConfigPath() {
        if (g_configPath.empty()) {
            // Build path: <game>/Data/SKSE/Plugins/VRClimbing.ini
            wchar_t pathBuf[MAX_PATH];
            GetModuleFileNameW(nullptr, pathBuf, MAX_PATH);

            std::filesystem::path gamePath(pathBuf);
            gamePath = gamePath.parent_path();  // Remove exe name
            gamePath /= "Data";
            gamePath /= "SKSE";
            gamePath /= "Plugins";
            gamePath /= "VRClimbing.ini";

            g_configPath = gamePath.string();
            spdlog::info("Config: Path set to {}", g_configPath);
        }
        return g_configPath;
    }

    bool SetSettingDouble(const std::string_view& name, double val) {
        if (auto it = floatMap.find(name); it != floatMap.end()) {
            *it->second = static_cast<float>(val);
            return true;
        }
        if (auto it = intMap.find(name); it != intMap.end()) {
            *it->second = static_cast<int>(val);
            return true;
        }
        if (auto it = boolMap.find(name); it != boolMap.end()) {
            *it->second = (val != 0.0);
            return true;
        }
        return false;
    }

    bool GetSettingDouble(const std::string_view& name, double& out) {
        if (auto it = floatMap.find(name); it != floatMap.end()) {
            out = static_cast<double>(*it->second);
            return true;
        }
        if (auto it = intMap.find(name); it != intMap.end()) {
            out = static_cast<double>(*it->second);
            return true;
        }
        if (auto it = boolMap.find(name); it != boolMap.end()) {
            out = *it->second ? 1.0 : 0.0;
            return true;
        }
        return false;
    }
}
