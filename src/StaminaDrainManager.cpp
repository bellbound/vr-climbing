#include "StaminaDrainManager.h"
#include "EquipmentManager.h"
#include "Config.h"
#include <algorithm>
#include <spdlog/spdlog.h>

StaminaDrainManager* StaminaDrainManager::GetSingleton()
{
    static StaminaDrainManager instance;
    return &instance;
}

float StaminaDrainManager::CalculateStaminaCostPerSecond() const
{
    auto* equipMgr = EquipmentManager::GetSingleton();

    // Beast forms have no stamina drain
    if (equipMgr->IsInBeastForm()) {
        return 0.0f;
    }

    // God mode = no drain
    if (RE::PlayerCharacter::IsGodMode()) {
        return 0.0f;
    }

    // Overencumbered takes priority (if enabled)
    if (Config::options.overencumberedEnabled && equipMgr->IsOverEncumbered()) {
        return Config::options.overencumberedStaminaCost;
    }

    // Weight-based scaling (uses skill-scaled weight)
    if (Config::options.weightScalingEnabled) {
        float armorWeight = equipMgr->GetTotalArmorWeightSkillScaled();
        float maxWeight = Config::options.maxArmorWeight;

        // Calculate weight ratio (0.0 = naked, 1.0 = max armor)
        float weightRatio = std::clamp(armorWeight / maxWeight, 0.0f, 1.0f);

        // Interpolate between base and max stamina cost
        float baseCost = Config::options.baseStaminaCost;
        float maxCost = Config::options.maxStaminaCost;

        return baseCost + (maxCost - baseCost) * weightRatio;
    }

    // No weight scaling - use base cost
    return Config::options.baseStaminaCost;
}

bool StaminaDrainManager::UpdateClimbingDrain(float deltaTime)
{
    float staminaCost = CalculateStaminaCostPerSecond();
    float minStamina = Config::options.minStamina;
    if (minStamina < 0.0f) {
        minStamina = 0.0f;
    }

    // If stamina cost is 0, no drain needed
    if (staminaCost <= 0.0f) {
        return true;
    }

    // Accumulate time - only drain periodically to reduce overhead
    m_accumulatedTime += deltaTime;
    if (m_accumulatedTime < DRAIN_INTERVAL) {
        return true;  // Not time to drain yet
    }

    // Drain stamina for accumulated time
    float drainAmount = staminaCost * m_accumulatedTime;
    m_accumulatedTime = 0.0f;

    DrainStamina(drainAmount);

    // Check if out of stamina
    if (GetCurrentStamina() <= minStamina) {
        spdlog::info("StaminaDrainManager: Out of stamina - dropping!");
        return false;  // Force release
    }

    return true;  // Can continue climbing
}

bool StaminaDrainManager::CanStartClimbing() const
{
    // Beast forms and god mode can always climb
    if (EquipmentManager::GetSingleton()->IsInBeastForm() || RE::PlayerCharacter::IsGodMode()) {
        return true;
    }

    // If stamina cost is 0, can always climb
    if (CalculateStaminaCostPerSecond() <= 0.0f) {
        return true;
    }

    // Just need stamina > 0 to start climbing
    float minStamina = Config::options.minStamina;
    if (minStamina < 0.0f) {
        minStamina = 0.0f;
    }
    
    return GetCurrentStamina() > minStamina;
}

void StaminaDrainManager::OnClimbingStopped()
{
    m_accumulatedTime = 0.0f;
}

float StaminaDrainManager::GetStaminaPercent() const
{
    float max = GetMaxStamina();
    if (max <= 0.0f) {
        return 0.0f;
    }
    return GetCurrentStamina() / max;
}

void StaminaDrainManager::DrainStamina(float amount)
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return;
    }

    player->AsActorValueOwner()->RestoreActorValue(
        RE::ACTOR_VALUE_MODIFIER::kDamage,
        RE::ActorValue::kStamina,
        -amount  // Negative to drain
    );
}

float StaminaDrainManager::GetCurrentStamina() const
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return 0.0f;
    }

    return player->AsActorValueOwner()->GetActorValue(RE::ActorValue::kStamina);
}

float StaminaDrainManager::GetMaxStamina() const
{
    auto* player = RE::PlayerCharacter::GetSingleton();
    if (!player) {
        return 0.0f;
    }

    return player->AsActorValueOwner()->GetPermanentActorValue(RE::ActorValue::kStamina);
}

