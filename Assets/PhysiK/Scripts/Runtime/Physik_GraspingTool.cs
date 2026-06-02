using System;
using System.Collections.Generic;
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
    [SerializeField] private int maxAttachments = 4;
    [SerializeField] private float graspRadiusMultiplier = 1.5f;
    [SerializeField] private float graspStiffness = 20000.0f;
    [SerializeField] private float graspDamping = 0.0f;

    [Header("Physical Interaction")]
    [SerializeField] private float collisionConnectionStiffness = 10000.0f;
    [SerializeField] private float collisionConnectionDamping = 0.0f;

    private readonly List<GraspAttachment> attachments =
        new List<GraspAttachment>();

    private PhysiKComponentHandle sphereComponent;
    private Physik_World physikWorld;
    private bool initialized;
    private bool possessed;
    private bool grasping;

    public bool IsInitialized => initialized;

    public bool IsPossessed => possessed;

    public bool IsGrasping => grasping;

    public int AttachmentCount => attachments.Count;

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

        if (toggleGrasp)
        {
            if (grasping)
            {
                Release();
            }
            else
            {
                TryBeginGrasp();
            }
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
            tissue == null ||
            tissue.WorldHandle == IntPtr.Zero)
        {
            return;
        }

        PushActiveGraspConnections();
    }

    public void OnPhysikAfterSimulationFrame()
    {
    }

    public void OnPhysikWorldDestroyed()
    {
        attachments.Clear();
        grasping = false;
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

    private void TryBeginGrasp()
    {
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

            return;
        }

        float graspRadius =
            RadiusFromTransform *
            Mathf.Max(
                0.0f,
                graspRadiusMultiplier);

        float graspRadiusSquared =
            graspRadius *
            graspRadius;

        Vector3 toolPosition =
            transform.position;

        List<(int localNodeIndex, float distanceSquared)> candidates =
            new List<(int localNodeIndex, float distanceSquared)>();

        for (int localNodeIndex = 0;
             localNodeIndex < nodeWorldPositions.Length;
             ++localNodeIndex)
        {
            float distanceSquared =
                (nodeWorldPositions[localNodeIndex] -
                 toolPosition)
                .sqrMagnitude;

            if (distanceSquared <= graspRadiusSquared)
            {
                candidates.Add(
                    (localNodeIndex, distanceSquared));
            }
        }

        candidates.Sort(
            (a, b) =>
                a.distanceSquared.CompareTo(
                    b.distanceSquared));

        attachments.Clear();

        int attachmentCount =
            Mathf.Min(
                Mathf.Max(
                    1,
                    maxAttachments),
                candidates.Count);

        for (int i = 0;
             i < attachmentCount;
             ++i)
        {
            int localNodeIndex =
                candidates[i].localNodeIndex;

            attachments.Add(
                new GraspAttachment
                {
                    globalNodeIndex =
                        globalNodeIndices[localNodeIndex],

                    localOffsetFromTool =
                        transform.InverseTransformPoint(
                            nodeWorldPositions[localNodeIndex])
                });
        }

        grasping =
            attachments.Count > 0;

        if (!grasping)
        {
            Debug.Log(
                "Physik_GraspingTool found no tissue node inside its grasp radius.",
                this);
        }
    }

    private void PushActiveGraspConnections()
    {
        for (int i = 0;
             i < attachments.Count;
             ++i)
        {
            GraspAttachment attachment =
                attachments[i];

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
    }

    private void Release()
    {
        attachments.Clear();
        grasping = false;
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

        attachments.Clear();
        grasping = false;
        initialized = false;
    }
}
