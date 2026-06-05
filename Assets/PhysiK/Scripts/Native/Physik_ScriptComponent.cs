using System;
using UnityEngine;
using PhysiK.Unity;

public abstract class Physik_ScriptComponent :
    MonoBehaviour
{
    private PhysiKComponentHandle
        nativeScriptComponent;

    // Native code stores this function pointer.
    // Keep the delegate alive as a managed field.
    private PhysiKNative.ExternalLogicCallback
        nativeCallback;

    private bool nativeScriptComponentInitialized;

    protected bool IsNativeScriptComponentInitialized =>
        nativeScriptComponentInitialized;

    protected abstract IntPtr
        GetScriptWorldHandle();

    protected abstract bool
        CanInitializeScriptComponent();

    protected abstract void
        OnPhysikPreUpdate();

    protected virtual void
        OnNativeScriptComponentInitialized()
    {
    }

    protected bool
        TryInitializeNativeScriptComponent()
    {
        if (nativeScriptComponentInitialized)
        {
            return true;
        }

        if (!CanInitializeScriptComponent())
        {
            return false;
        }

        IntPtr world =
            GetScriptWorldHandle();

        if (world ==
            IntPtr.Zero)
        {
            return false;
        }

        nativeScriptComponent =
            PhysiKNative
                .PHYSIK_CreateScriptComponent(
                    world);

        if (PhysiKNative
                .PHYSIK_IsComponentHandleValid(
                    world,
                    nativeScriptComponent) ==
            0)
        {
            Debug.LogError(
                "Failed to create native ScriptComponent.",
                this);

            return false;
        }

        nativeCallback =
            OnNativePreUpdate;

        PhysiKNative
            .PHYSIK_SetScriptComponentCallback(
                world,
                nativeScriptComponent,
                nativeCallback,
                IntPtr.Zero);

        nativeScriptComponentInitialized =
            true;

        OnNativeScriptComponentInitialized();

        return true;
    }

    private void OnNativePreUpdate(
        IntPtr callbackWorld,
        IntPtr userData)
    {
        if (!nativeScriptComponentInitialized ||
            callbackWorld ==
                IntPtr.Zero ||
            callbackWorld !=
                GetScriptWorldHandle())
        {
            return;
        }

        OnPhysikPreUpdate();
    }

    protected void DestroyNativeScriptComponent()
    {
        if (!nativeScriptComponentInitialized)
        {
            return;
        }

        IntPtr world =
            GetScriptWorldHandle();

        if (world !=
            IntPtr.Zero)
        {
            PhysiKNative
                .PHYSIK_ClearScriptComponentCallback(
                    world,
                    nativeScriptComponent);

            PhysiKNative
                .PHYSIK_DestroyComponent(
                    world,
                    nativeScriptComponent);
        }

        nativeCallback =
            null;

        nativeScriptComponent =
            default;

        nativeScriptComponentInitialized =
            false;
    }

    protected virtual void OnDestroy()
    {
        DestroyNativeScriptComponent();
    }
}