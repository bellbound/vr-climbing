#pragma once

#include "RE/Skyrim.h"

// Manages stamina drain for climbing activities
// Simple rule: stamina > 0 = can climb, stamina = 0 = drop
// Configuration is in Config::options (staminaDrainEnabled, staminaDrainPerSecond, minStaminaToClimb)
class StaminaDrainManager
{
public:
    static StaminaDrainManager* GetSingleton();

    // Called periodically while climbing - drains stamina
    // Returns false if stamina depleted (player should drop)
    bool UpdateClimbingDrain(float deltaTime);

    // Check if player can start climbing (stamina > 0)
    bool CanStartClimbing() const;

    // Called when climbing stops (resets accumulator)
    void OnClimbingStopped();

    // Get current stamina as percentage (0.0 - 1.0)
    float GetStaminaPercent() const;

    // Calculate current stamina cost per second based on equipment weight
    float CalculateStaminaCostPerSecond() const;

private:
    StaminaDrainManager() = default;
    ~StaminaDrainManager() = default;
    StaminaDrainManager(const StaminaDrainManager&) = delete;
    StaminaDrainManager& operator=(const StaminaDrainManager&) = delete;

    void DrainStamina(float amount);
    float GetCurrentStamina() const;
    float GetMaxStamina() const;

    // Throttling - accumulate time and only drain periodically
    float m_accumulatedTime = 0.0f;
    static constexpr float DRAIN_INTERVAL = 0.1f;  // Drain every 100ms
};
