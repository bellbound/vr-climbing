#include "HiggsCompatManager.h"
#include "higgsinterface001.h"
#include <spdlog/spdlog.h>
#include <string>

HiggsCompatManager* HiggsCompatManager::GetSingleton()
{
    static HiggsCompatManager instance;
    return &instance;
}

void HiggsCompatManager::Initialize()
{
    if (m_initialized) {
        return;
    }

    if (!g_higgsInterface) {
        spdlog::warn("HiggsCompatManager: HIGGS interface not available");
        return;
    }

    // Read and store the original HIGGS settings
    if (g_higgsInterface->GetSettingDouble("disableGravityGlovesLooting", m_originalDisableGravityGloves)) {
        spdlog::info("HiggsCompatManager: Original disableGravityGlovesLooting = {}", m_originalDisableGravityGloves);
    } else {
        spdlog::warn("HiggsCompatManager: Failed to read disableGravityGlovesLooting, assuming 0 (enabled)");
        m_originalDisableGravityGloves = 0.0;
    }

    if (g_higgsInterface->GetSettingDouble("enableTwoHandedGrabbing", m_originalEnableTwoHandedGrabbing)) {
        spdlog::info("HiggsCompatManager: Original enableTwoHandedGrabbing = {}", m_originalEnableTwoHandedGrabbing);
    } else {
        spdlog::warn("HiggsCompatManager: Failed to read enableTwoHandedGrabbing, assuming 1 (enabled)");
        m_originalEnableTwoHandedGrabbing = 1.0;
    }

    if (g_higgsInterface->GetSettingDouble("enableWeaponTwoHanding", m_originalEnableWeaponTwoHanding)) {
        spdlog::info("HiggsCompatManager: Original enableWeaponTwoHanding = {}", m_originalEnableWeaponTwoHanding);
    } else {
        spdlog::warn("HiggsCompatManager: Failed to read enableWeaponTwoHanding, assuming 1 (enabled)");
        m_originalEnableWeaponTwoHanding = 1.0;
    }

    if (g_higgsInterface->GetSettingDouble("EnableGrip", m_originalEnableGrip)) {
        spdlog::info("HiggsCompatManager: Original EnableGrip = {}", m_originalEnableGrip);
    } else {
        spdlog::warn("HiggsCompatManager: Failed to read EnableGrip, assuming 1 (enabled)");
        m_originalEnableGrip = 1.0;
    }

    m_initialized = true;
    spdlog::info("HiggsCompatManager: Initialized");
}

void HiggsCompatManager::DisableGravityGloves()
{
    if (!m_initialized || !g_higgsInterface) {
        return;
    }

    // Increment counter to cancel any pending restore
    ++m_disableRequestCounter;
    m_restorePending = false;

    // Disable gravity gloves (if not already disabled by user)
    if (!m_gravityGlovesDisabledByUs && m_originalDisableGravityGloves == 0.0) {
        if (g_higgsInterface->SetSettingDouble("disableGravityGlovesLooting", 1.0)) {
            m_gravityGlovesDisabledByUs = true;
        }
    }

    // Disable two-handed grabbing (if not already disabled by user)
    if (!m_twoHandedGrabbingDisabledByUs && m_originalEnableTwoHandedGrabbing != 0.0) {
        if (g_higgsInterface->SetSettingDouble("enableTwoHandedGrabbing", 0.0)) {
            m_twoHandedGrabbingDisabledByUs = true;
        }
    }

    // Disable weapon two-handing (if not already disabled by user)
    if (!m_weaponTwoHandingDisabledByUs && m_originalEnableWeaponTwoHanding != 0.0) {
        if (g_higgsInterface->SetSettingDouble("enableWeaponTwoHanding", 0.0)) {
            m_weaponTwoHandingDisabledByUs = true;
        }
    }

    // Disable grip (if not already disabled by user and nothing is held)
    // If either hand is holding something with HIGGS, leave grip enabled
    bool leftHolding = g_higgsInterface->IsHoldingObject(true);
    bool rightHolding = g_higgsInterface->IsHoldingObject(false);
    if (!m_gripDisabledByUs && m_originalEnableGrip != 0.0 && !leftHolding && !rightHolding) {
        if (g_higgsInterface->SetSettingDouble("EnableGrip", 0.0)) {
            m_gripDisabledByUs = true;
        }
    }
}

void HiggsCompatManager::RestoreGravityGloves()
{
    if (!m_initialized || !g_higgsInterface) {
        return;
    }

    // Schedule a delayed restore instead of restoring immediately
    // This prevents flickering when quickly re-grabbing
    m_restorePending = true;
    m_restoreRequestId = m_disableRequestCounter;
    m_restoreRequestTime = std::chrono::steady_clock::now();
}

void HiggsCompatManager::Update()
{
    if (!m_restorePending) {
        return;
    }

    // Check if a new disable request came in since we scheduled the restore
    if (m_restoreRequestId != m_disableRequestCounter) {
        // A new disable was requested - cancel the pending restore
        m_restorePending = false;
        spdlog::debug("HiggsCompatManager: Restore cancelled - new disable request came in");
        return;
    }

    // Check if enough time has passed
    auto now = std::chrono::steady_clock::now();
    float elapsedMs = std::chrono::duration<float, std::milli>(now - m_restoreRequestTime).count();

    if (elapsedMs >= RESTORE_DELAY_MS) {
        m_restorePending = false;
        DoRestore();
    }
}

void HiggsCompatManager::DoRestore()
{
    if (!g_higgsInterface) {
        return;
    }

    std::string restored;

    // Restore gravity gloves (only if we disabled it)
    if (m_gravityGlovesDisabledByUs) {
        if (g_higgsInterface->SetSettingDouble("disableGravityGlovesLooting", m_originalDisableGravityGloves)) {
            m_gravityGlovesDisabledByUs = false;
            restored += "gravityGloves ";
        }
    }

    // Restore two-handed grabbing (only if we disabled it)
    if (m_twoHandedGrabbingDisabledByUs) {
        if (g_higgsInterface->SetSettingDouble("enableTwoHandedGrabbing", m_originalEnableTwoHandedGrabbing)) {
            m_twoHandedGrabbingDisabledByUs = false;
            restored += "twoHandedGrabbing ";
        }
    }

    // Restore weapon two-handing (only if we disabled it)
    if (m_weaponTwoHandingDisabledByUs) {
        if (g_higgsInterface->SetSettingDouble("enableWeaponTwoHanding", m_originalEnableWeaponTwoHanding)) {
            m_weaponTwoHandingDisabledByUs = false;
            restored += "weaponTwoHanding ";
        }
    }

    // Restore grip (only if we disabled it)
    if (m_gripDisabledByUs) {
        if (g_higgsInterface->SetSettingDouble("EnableGrip", m_originalEnableGrip)) {
            m_gripDisabledByUs = false;
            restored += "grip ";
        }
    }

 
}
