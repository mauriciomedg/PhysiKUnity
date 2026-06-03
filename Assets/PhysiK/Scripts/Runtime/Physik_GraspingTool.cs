using System;
using UnityEngine;
using UnityEngine.InputSystem;
using PhysiK.Unity;

[DefaultExecutionOrder(-200)]
public sealed class Physik_GraspingTool : MonoBehaviour, IPhysikWorldParticipant
{
    private struct GraspAttachment
    {
        public int globalNodeIndex;
        public Vector3 localOffsetFromTool;
    }

    [Header("Mechanical Tissue")]
    [SerializeField] private Physik_MechanicalTissue tissue;

    [Header("Grasping")]
    [SerializeField] private float graspStiffness = 20000.0f;
    [SerializeField] private float graspDamping = 0.0f;

    [Header("Physical Interaction")]
    [SerializeField] private float collisionConnectionStiffness = 10000.0f;
    [SerializeField] private float collisionConnectionDamping = 0.0f;

    private PhysiKComponentHandle sphereComponent;
    private Physik_World physikWorld;

    private GraspAttachment attachment;
    private bool hasAttachment;

    private bool initialized;
    private bool possessed;
    private bool grasping;

    public bool IsInitialized => initialized;

    public bool IsPossessed => possessed;

    public bool IsGrasping => grasping;

    public int AttachmentCount => hasAttachment ? 1 : 0;

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
            "Physik_GraspingTool";
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

        if (!initialized)
        {
            return;
        }

        ApplyCollisionConnectionSettings();
        PushNativeSpherePosition();

        bool toggleGrasp =
            possessed &&
            Keyboard.current != null &&
            Keyboard.current.gKey.wasPressedThisFrame;

        if (!toggleGrasp)
        {
            return;
        }

        if (grasping)
        {
            Release();
        }
        else
        {
            TryBeginGrasp();
        }
    }

    public void SetPossessed(bool isPossessed)
    {
        possessed =
            isPossessed;
    }

    public void OnPhysikBeforeSimulationStep(float dt)
    {
        if (!initialized ||
            !grasping ||
            !hasAttachment ||
            tissue == null ||
            tissue.WorldHandle == IntPtr.Zero)
        {
            return;
        }

        PushActiveGraspConnection();
    }

    public void OnPhysikAfterSimulationFrame()
    {
    }

    public void OnPhysikWorldDestroyed()
    {
        Release();
        initialized = false;
        physikWorld = null;
    }

    private bool TryInitialize()
    {
        if (initialized)
        {
            return true;
        }

        if (tissue == null)
        {
            tissue =
                FindFirstObjectByType<Physik_MechanicalTissue>();
        }

        if (tissue == null ||
            !tissue.IsInitialized ||
            tissue.WorldHandle == IntPtr.Zero)
        {
            return false;
        }

        physikWorld =
            tissue.WorldOwner;

        if (physikWorld == null)
        {
            Debug.LogError(
                "Physik_GraspingTool could not find the owning Physik_World.",
                this);

            enabled = false;
            return false;
        }

        Vector3 position =
            transform.position;

        sphereComponent =
            PhysiKNative.PHYSIK_CreateCollisionSphereComponent(
                tissue.WorldHandle,
                position.x,
                position.y,
                position.z,
                RadiusFromTransform);

        initialized =
            PhysiKNative.PHYSIK_IsComponentHandleValid(
                tissue.WorldHandle,
                sphereComponent) != 0;

        if (!initialized)
        {
            Debug.LogError(
                "Failed to create native CollisionSphereComponent for Physik_GraspingTool.",
                this);

            enabled = false;
            return false;
        }

        ApplyCollisionConnectionSettings();

        physikWorld.RegisterParticipant(
            this);

        return true;
    }

    private void ApplyCollisionConnectionSettings()
    {
        if (!initialized ||
            tissue == null ||
            tissue.WorldHandle == IntPtr.Zero)
        {
            return;
        }

        PhysiKNative.PHYSIK_SetCollisionSphereConnectionSettings(
            tissue.WorldHandle,
            sphereComponent,
            collisionConnectionStiffness,
            collisionConnectionDamping);
    }

    private void PushNativeSpherePosition()
    {
        Vector3 position =
            transform.position;

        PhysiKNative.PHYSIK_SetCollisionComponentKinematicTarget(
            tissue.WorldHandle,
            sphereComponent,
            position.x,
            position.y,
            position.z);
    }

    private bool TryFindClosestNodeInsideTool(
    out int closestLocalNodeIndex)
    {
        closestLocalNodeIndex =
            -1;

        int[] globalNodeIndices =
            tissue.GlobalNodeIndices;

        Vector3[] nodeWorldPositions =
            tissue.NodeWorldPositions;

        if (globalNodeIndices == null ||
            nodeWorldPositions == null ||
            globalNodeIndices.Length != nodeWorldPositions.Length)
        {
            Debug.LogWarning(
                "Physik_GraspingTool could not read the tissue nodes.",
                this);

            return false;
        }

        Vector3 toolPosition =
            transform.position;

        float graspRadiusSquared =
            RadiusFromTransform *
            RadiusFromTransform;

        float closestDistanceSquared =
            float.PositiveInfinity;

        for (int localNodeIndex = 0;
             localNodeIndex < nodeWorldPositions.Length;
             ++localNodeIndex)
        {
            float distanceSquared =
                (nodeWorldPositions[localNodeIndex] -
                 toolPosition)
                .sqrMagnitude;

            if (distanceSquared > graspRadiusSquared ||
                distanceSquared >= closestDistanceSquared)
            {
                continue;
            }

            closestDistanceSquared =
                distanceSquared;

            closestLocalNodeIndex =
                localNodeIndex;
        }

        return closestLocalNodeIndex >= 0;
    }

    private void CreateAttachment(
    int localNodeIndex)
    {
        int[] globalNodeIndices =
            tissue.GlobalNodeIndices;

        Vector3[] nodeWorldPositions =
            tissue.NodeWorldPositions;

        attachment =
            new GraspAttachment
            {
                globalNodeIndex =
                    globalNodeIndices[localNodeIndex],

                localOffsetFromTool =
                    transform.InverseTransformPoint(
                        nodeWorldPositions[localNodeIndex])
            };

        hasAttachment =
            true;
    }
    private void TryBeginGrasp()
    {
        if (!TryFindClosestNodeInsideTool(
                out int closestLocalNodeIndex))
        {
            Debug.Log(
                "Physik_GraspingTool found no tissue node inside the tool sphere.",
                this);

            return;
        }

        CreateAttachment(
            closestLocalNodeIndex);

        grasping =
            true;
    }

    private void PushActiveGraspConnection()
    {
        Vector3 target =
            transform.TransformPoint(
                attachment.localOffsetFromTool);

        PhysiKNative.PHYSIK_AddPointConnection(
            tissue.WorldHandle,
            attachment.globalNodeIndex,
            attachment.globalNodeIndex,
            attachment.globalNodeIndex,
            attachment.globalNodeIndex,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            target.x,
            target.y,
            target.z,
            graspStiffness,
            graspDamping);
    }

    private void Release()
    {
        hasAttachment =
            false;

        grasping =
            false;
    }

    private void OnDestroy()
    {
        if (physikWorld != null)
        {
            physikWorld.UnregisterParticipant(
                this);
        }

        if (initialized &&
            tissue != null &&
            tissue.WorldHandle != IntPtr.Zero)
        {
            PhysiKNative.PHYSIK_DestroyComponent(
                tissue.WorldHandle,
                sphereComponent);
        }

        Release();

        initialized =
            false;
    }
}
