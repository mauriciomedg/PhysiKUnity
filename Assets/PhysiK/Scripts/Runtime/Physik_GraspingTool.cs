using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;
using PhysiK.Unity;

[DefaultExecutionOrder(-200)]
public sealed class Physik_GraspingTool :
    MonoBehaviour,
    IPhysikWorldParticipant
{
    private struct GraspAttachment
    {
        public int globalNodeIndex;

        // Contact position expressed in tool-local coordinates.
        // Calculated once when grasping begins.
        public Vector3 localOffsetFromTool;
    }

    private struct ContactCandidate
    {
        public int localNodeIndex;

        // Smaller values mean that the node is closer
        // to the visible surface of the sphere.
        public float distanceToSphereSurface;
    }

    [Header("Mechanical Tissue")]
    [SerializeField]
    private Physik_MechanicalTissue tissue;

    [Header("Grasping")]
    [FormerlySerializedAs("graspContactCount")]
    [SerializeField]
    [Min(1)]
    private int maxGraspContactCount =
        5;

    [SerializeField]
    private float graspStiffness =
        20000.0f;

    [SerializeField]
    private float graspDamping =
        0.0f;

    [Header("Physical Interaction")]
    [SerializeField]
    private float collisionConnectionStiffness =
        10000.0f;

    [SerializeField]
    private float collisionConnectionDamping =
        0.0f;

    private PhysiKComponentHandle sphereComponent;

    private Physik_World physikWorld;

    private readonly List<GraspAttachment> attachments =
        new List<GraspAttachment>();

    private bool initialized;

    private bool possessed;

    private bool grasping;

    public bool IsInitialized =>
        initialized;

    public bool IsPossessed =>
        possessed;

    public bool IsGrasping =>
        grasping;

    public int AttachmentCount =>
        attachments.Count;

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

        maxGraspContactCount =
            Mathf.Max(
                1,
                maxGraspContactCount);
    }

    private void OnValidate()
    {
        maxGraspContactCount =
            Mathf.Max(
                1,
                maxGraspContactCount);
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

    public void SetPossessed(
        bool isPossessed)
    {
        possessed =
            isPossessed;
    }

    public void OnPhysikBeforeSimulationStep(
        float dt)
    {
        if (!initialized ||
            !grasping ||
            attachments.Count == 0 ||
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
        Release();

        initialized =
            false;

        physikWorld =
            null;
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
                FindFirstObjectByType<
                    Physik_MechanicalTissue>();
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

            enabled =
                false;

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
                sphereComponent) !=
            0;

        if (!initialized)
        {
            Debug.LogError(
                "Failed to create native CollisionSphereComponent for Physik_GraspingTool.",
                this);

            enabled =
                false;

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

    private bool TryFindClosestSurfaceContactsInsideTool(
        out List<int> selectedLocalNodeIndices)
    {
        selectedLocalNodeIndices =
            new List<int>();

        int[] globalNodeIndices =
            tissue.GlobalNodeIndices;

        Vector3[] nodeWorldPositions =
            tissue.NodeWorldPositions;

        if (globalNodeIndices == null ||
            nodeWorldPositions == null ||
            globalNodeIndices.Length !=
                nodeWorldPositions.Length)
        {
            Debug.LogWarning(
                "Physik_GraspingTool could not read the tissue nodes.",
                this);

            return false;
        }

        Vector3 sphereCenter =
            transform.position;

        float sphereRadius =
            RadiusFromTransform;

        float sphereRadiusSquared =
            sphereRadius *
            sphereRadius;

        List<ContactCandidate> candidates =
            new List<ContactCandidate>();

        for (int localNodeIndex = 0;
             localNodeIndex <
                 nodeWorldPositions.Length;
             ++localNodeIndex)
        {
            Vector3 offsetFromSphereCenter =
                nodeWorldPositions[localNodeIndex] -
                sphereCenter;

            float distanceSquared =
                offsetFromSphereCenter.sqrMagnitude;

            // Only consider nodes that are currently inside the sphere.
            if (distanceSquared >
                sphereRadiusSquared)
            {
                continue;
            }

            float distanceFromSphereCenter =
                Mathf.Sqrt(
                    distanceSquared);

            float distanceToSphereSurface =
                sphereRadius -
                distanceFromSphereCenter;

            candidates.Add(
                new ContactCandidate
                {
                    localNodeIndex =
                        localNodeIndex,

                    distanceToSphereSurface =
                        distanceToSphereSurface
                });
        }

        if (candidates.Count ==
            0)
        {
            Debug.Log(
                "Physik_GraspingTool found no tissue contacts inside the sphere.",
                this);

            return false;
        }

        candidates.Sort(
            (
                ContactCandidate left,
                ContactCandidate right
            ) =>
            left.distanceToSphereSurface.CompareTo(
                right.distanceToSphereSurface));

        int selectedContactCount =
            Mathf.Min(
                candidates.Count,
                maxGraspContactCount);

        for (int contactIndex = 0;
             contactIndex <
                 selectedContactCount;
             ++contactIndex)
        {
            selectedLocalNodeIndices.Add(
                candidates[contactIndex]
                    .localNodeIndex);
        }

        return selectedLocalNodeIndices.Count >
            0;
    }

    private void CreateAttachments(
        List<int> localNodeIndices)
    {
        attachments.Clear();

        int[] globalNodeIndices =
            tissue.GlobalNodeIndices;

        Vector3[] nodeWorldPositions =
            tissue.NodeWorldPositions;

        Matrix4x4 inverseToolTransform =
            transform.worldToLocalMatrix;

        foreach (int localNodeIndex
                 in localNodeIndices)
        {
            Vector3 selectedContactWorldPosition =
                nodeWorldPositions[
                    localNodeIndex];

            attachments.Add(
                new GraspAttachment
                {
                    globalNodeIndex =
                        globalNodeIndices[
                            localNodeIndex],

                    localOffsetFromTool =
                        inverseToolTransform
                            .MultiplyPoint3x4(
                                selectedContactWorldPosition)
                });
        }
    }

    private void TryBeginGrasp()
    {
        if (!TryFindClosestSurfaceContactsInsideTool(
                out List<int> selectedLocalNodeIndices))
        {
            return;
        }

        CreateAttachments(
            selectedLocalNodeIndices);

        grasping =
            attachments.Count >
            0;

        if (grasping)
        {
            Debug.Log(
                $"Physik_GraspingTool grasped tissue with {attachments.Count} contact(s).",
                this);
        }
    }

    private void PushActiveGraspConnections()
    {
        Matrix4x4 toolTransform =
            transform.localToWorldMatrix;

        foreach (GraspAttachment attachment
                 in attachments)
        {
            Vector3 targetWorldPosition =
                toolTransform
                    .MultiplyPoint3x4(
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
                targetWorldPosition.x,
                targetWorldPosition.y,
                targetWorldPosition.z,
                graspStiffness,
                graspDamping);
        }
    }

    private void Release()
    {
        attachments.Clear();

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
