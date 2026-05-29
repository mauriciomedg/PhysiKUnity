using System;
using UnityEngine;
using UnityEngine.InputSystem;
using PhysiK.Unity;
using static PhysiKNative;
using UnityEngine.LightTransport;

public sealed class PhysikTool : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Physik_MechanicalTissue tissue;
    [SerializeField] private Camera sceneCamera;

    [Header("Tool")]
    [SerializeField] private float heightOffset = 0.0f;
    [SerializeField] private bool cutWhileMouseHeld = true;
    [SerializeField] private bool cutContinuously = true;
    [SerializeField] private int maxCutsPerFrame = 8;

    [Header("Physical Interaction")]
    [SerializeField] private float connectionStiffness = 10000.0f;
    [SerializeField] private float connectionDamping = 0.0f;

    [Header("Visual")]
    [SerializeField] private MeshRenderer visualRenderer;

    private PhysiKComponentHandle sphereComponent;
    private bool hasNativeSphere;

    private float RadiusFromTransform
    {
        get
        {
            Vector3 scale = transform.lossyScale;

            // Unity sphere primitive has diameter 1 at scale (1,1,1),
            // so radius = max scale axis * 0.5.
            return Mathf.Max(scale.x, scale.y, scale.z) * 0.5f;
        }
    }

    private void Awake()
    {
        if (sceneCamera == null)
        {
            sceneCamera = Camera.main;
        }

        if (visualRenderer == null)
        {
            visualRenderer = GetComponentInChildren<MeshRenderer>();
        }

        gameObject.name = "Physik_Tool";
    }

    private void Start()
    {
        if (tissue == null)
        {
            tissue = FindFirstObjectByType<Physik_MechanicalTissue>();
        }

        if (tissue == null || tissue.WorldHandle == IntPtr.Zero)
        {
            UnityEngine.Debug.LogError("PhysikTool could not find a valid PhysikCircularTissue/world.", this);
            enabled = false;
            return;
        }

        Vector3 p = transform.position;

        float radius = RadiusFromTransform;

        sphereComponent = PhysiKNative.PHYSIK_CreateCollisionSphereComponent(
            tissue.WorldHandle,
            p.x,
            p.y,
            p.z,
            radius);

        hasNativeSphere = PhysiKNative.PHYSIK_IsComponentHandleValid(
            tissue.WorldHandle,
            sphereComponent) != 0;

        if (!hasNativeSphere)
        {
            UnityEngine.Debug.LogError("Failed to create native CollisionSphereComponent for PhysikTool.", this);
            enabled = false;
            return;
        }

        PhysiKNative.PHYSIK_SetCollisionSphereConnectionSettings(
            tissue.WorldHandle,
            sphereComponent,
            connectionStiffness,
            connectionDamping);
    }

    private void ApplyConnectionSettings()
    {
        if (!hasNativeSphere || tissue == null || tissue.WorldHandle == IntPtr.Zero)
            return;

        PhysiKNative.PHYSIK_SetCollisionSphereConnectionSettings(
            tissue.WorldHandle,
            sphereComponent,
            connectionStiffness,
            connectionDamping);
    }
    private void Update()
    {
        if (!hasNativeSphere || tissue == null || tissue.WorldHandle == IntPtr.Zero)
        {
            return;
        }

        ApplyConnectionSettings();
        UpdateMousePosition();
        PushNativeSpherePosition();

        bool isCPressed =
        Keyboard.current != null &&
        Keyboard.current.cKey.isPressed;

        bool shouldCut =
            isCPressed &&
            (
                cutContinuously ||
                (cutWhileMouseHeld && Mouse.current != null && Mouse.current.leftButton.isPressed)
            );

        if (shouldCut)
        {
            CutOverlappingTets();
        }
    }

    private void UpdateMousePosition()
    {
        if (sceneCamera == null || Mouse.current == null)
        {
            return;
        }

        Vector2 mousePosition = Mouse.current.position.ReadValue();
        Ray ray = sceneCamera.ScreenPointToRay(mousePosition);

        Plane plane = new Plane(
            Vector3.up,
            new Vector3(0.0f, tissue.TissuePlaneY + heightOffset, 0.0f));

        if (plane.Raycast(ray, out float distance))
        {
            transform.position = ray.GetPoint(distance);
        }
    }

    private void PushNativeSpherePosition()
    {
        Vector3 p = transform.position;

        PhysiKNative.PHYSIK_SetCollisionComponentKinematicTarget(
            tissue.WorldHandle,
            sphereComponent,
            p.x,
            p.y,
            p.z);
    }
    
    private void CutOverlappingTets()
    {
        int count = PhysiKNative.PHYSIK_GetCollisionSphereOverlapCount(
            tissue.WorldHandle,
            sphereComponent);

        if (count <= 0)
        {
            return;
        }

        PhysikCollisionSphereOverlap[] overlaps =
            new PhysikCollisionSphereOverlap[count];

        int written = PhysiKNative.PHYSIK_GetCollisionSphereOverlaps(
            tissue.WorldHandle,
            sphereComponent,
            overlaps,
            overlaps.Length);

        int cuts = 0;

        for (int i = 0; i < written; ++i)
        {
            if ((PhysikOverlapGeometryType)overlaps[i].geometryType !=
                PhysikOverlapGeometryType.Tetrahedron)
            {
                continue;
            }

            if (!SameHandle(overlaps[i].component, tissue.TetMeshHandle))
            {
                continue;
            }

            tissue.DeactivateTet(overlaps[i].primitiveIndex);
        }
    }

    private static bool SameHandle(PhysiKComponentHandle a, PhysiKComponentHandle b)
    {
        return a.index == b.index && a.generation == b.generation;
    }

    private void OnDestroy()
    {
        if (hasNativeSphere && tissue != null && tissue.WorldHandle != IntPtr.Zero)
        {
            PhysiKNative.PHYSIK_DestroyComponent(tissue.WorldHandle, sphereComponent);
            hasNativeSphere = false;
        }
    }
}