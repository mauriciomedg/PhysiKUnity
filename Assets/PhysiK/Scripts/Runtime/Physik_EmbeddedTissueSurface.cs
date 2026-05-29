using System.Collections;
using UnityEngine;
using PhysiK.Unity;

public class Physik_EmbeddedTissueSurface : MonoBehaviour
{
    [Header("Embedded Tissue")]
    [SerializeField] private Physik_EmbeddedTissue embeddedTissue;

    [Header("Debug")]
    [SerializeField] private bool logCreation = true;

    private PhysiKComponentHandle surfaceExtractionHandle;
    private bool initialized;

    public bool IsInitialized => initialized;

    public Physik_EmbeddedTissue EmbeddedTissue => embeddedTissue;

    public System.IntPtr WorldHandle => embeddedTissue != null
        ? embeddedTissue.WorldHandle
        : System.IntPtr.Zero;

    public PhysiKComponentHandle SurfaceExtractionHandle => surfaceExtractionHandle;

    private IEnumerator Start()
    {
        if (embeddedTissue == null)
        {
            Debug.LogError("Missing embeddedTissue.", this);
            yield break;
        }

        yield return new WaitUntil(() => embeddedTissue.IsInitialized);

        CreateSurfaceExtractionComponent();
    }

    private void CreateSurfaceExtractionComponent()
    {
        if (embeddedTissue.WorldHandle == System.IntPtr.Zero)
        {
            Debug.LogError("Embedded tissue has invalid world handle.", this);
            initialized = false;
            return;
        }

        if (embeddedTissue.MappedTetMeshHandle.IsValid == false)
        {
            Debug.LogError("Embedded tissue has invalid mapped tet mesh handle.", this);
            initialized = false;
            return;
        }

        surfaceExtractionHandle = PhysiKNative.PHYSIK_CreateSurfaceExtractionComponent(
            embeddedTissue.WorldHandle,
            embeddedTissue.MappedTetMeshHandle);

        if (PhysiKNative.PHYSIK_IsComponentHandleValid(
                embeddedTissue.WorldHandle,
                surfaceExtractionHandle) == 0)
        {
            Debug.LogError("Failed to create SurfaceExtractionComponent.", this);
            initialized = false;
            return;
        }

        initialized = true;

        if (logCreation)
        {
            Debug.Log("Embedded tissue surface extraction component created.", this);
        }
    }
}