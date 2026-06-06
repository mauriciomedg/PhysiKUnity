using System;
using UnityEngine;
using PhysiK.Unity;

public sealed class Physik_GiantCollisionSphereFloor :
    MonoBehaviour
{
    [Header("PhysiK World")]
    [SerializeField]
    private Physik_World physikWorld;

    [Header("Native Collision")]
    [SerializeField]
    [Min(0.0f)]
    private float collisionConnectionStiffness =
        100000.0f;

    [SerializeField]
    [Min(0.0f)]
    private float collisionConnectionDamping =
        1000.0f;

    private PhysiKComponentHandle sphereComponent;

    private bool initialized;

    private IntPtr world =
        IntPtr.Zero;

    private float RadiusFromTransform
    {
        get
        {
            Vector3 scale =
                transform.lossyScale;

            return Mathf.Max(
                       scale.x,
                       scale.y,
                       scale.z) *
                   0.5f;
        }
    }

    private void Awake()
    {
        gameObject.name =
            "Physik_Floor";

        Collider unityCollider =
            GetComponent<
                Collider>();

        if (unityCollider !=
            null)
        {
            unityCollider.enabled =
                false;
        }
    }

    private void Start()
    {
        TryInitialize();
    }

    private void Update()
    {
        if (!initialized)
        {
            TryInitialize();
        }
    }

    private void TryInitialize()
    {
        if (initialized)
        {
            return;
        }

        if (physikWorld ==
            null)
        {
            physikWorld =
                FindFirstObjectByType<
                    Physik_World>();
        }

        if (physikWorld ==
            null)
        {
            return;
        }

        world =
            physikWorld.WorldHandle;

        if (world ==
            IntPtr.Zero)
        {
            return;
        }

        Vector3 center =
            transform.position;

        sphereComponent =
            PhysiKNative
                .PHYSIK_CreateCollisionSphereComponent(
                    world,
                    center.x,
                    center.y,
                    center.z,
                    RadiusFromTransform);

        initialized =
            PhysiKNative
                .PHYSIK_IsComponentHandleValid(
                    world,
                    sphereComponent) !=
            0;

        if (!initialized)
        {
            Debug.LogError(
                "Failed to create native floor collision sphere.",
                this);

            enabled =
                false;

            return;
        }

        PhysiKNative
            .PHYSIK_SetCollisionSphereConnectionSettings(
                world,
                sphereComponent,
                collisionConnectionStiffness,
                collisionConnectionDamping);

        Debug.Log(
            $"Physik floor created. " +
            $"radius={RadiusFromTransform}, " +
            $"center={center}.",
            this);
    }

    private void OnValidate()
    {
        collisionConnectionStiffness =
            Mathf.Max(
                0.0f,
                collisionConnectionStiffness);

        collisionConnectionDamping =
            Mathf.Max(
                0.0f,
                collisionConnectionDamping);
    }

    private void OnDestroy()
    {
        if (initialized &&
            world !=
                IntPtr.Zero)
        {
            PhysiKNative
                .PHYSIK_DestroyComponent(
                    world,
                    sphereComponent);
        }

        initialized =
            false;

        world =
            IntPtr.Zero;
    }
}