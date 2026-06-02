using UnityEngine;
using UnityEngine.InputSystem;

[DefaultExecutionOrder(-300)]
public sealed class Physik_ToolController : MonoBehaviour
{
    public enum ToolMode
    {
        Grasping,
        Cut
    }

    [Header("References")]
    [SerializeField] private Camera sceneCamera;
    [SerializeField] private Physik_GraspingTool graspingTool;
    [SerializeField] private Physik_CutTool cutTool;

    [Header("Possession")]
    [SerializeField] private ToolMode initialMode = ToolMode.Grasping;

    [Header("Vertical Movement")]
    [SerializeField] private float verticalSpeed = 1.5f;

    private ToolMode currentMode;

    public ToolMode CurrentMode => currentMode;

    private void Awake()
    {
        if (sceneCamera == null)
        {
            sceneCamera = Camera.main;
        }

        currentMode = initialMode;
        ApplyPossessionState();
    }

    private void Update()
    {
        ReadModeSelection();
        MovePossessedTool();
    }

    private void ReadModeSelection()
    {
        if (Keyboard.current == null)
        {
            return;
        }

        if (Keyboard.current.digit1Key.wasPressedThisFrame)
        {
            SelectMode(
                ToolMode.Grasping);
        }

        if (Keyboard.current.digit2Key.wasPressedThisFrame)
        {
            SelectMode(
                ToolMode.Cut);
        }
    }

    public void SelectMode(ToolMode mode)
    {
        if (currentMode == mode)
        {
            return;
        }

        currentMode = mode;
        ApplyPossessionState();
    }

    private void ApplyPossessionState()
    {
        if (graspingTool != null)
        {
            graspingTool.SetPossessed(
                currentMode == ToolMode.Grasping);
        }

        if (cutTool != null)
        {
            cutTool.SetPossessed(
                currentMode == ToolMode.Cut);
        }
    }

    private void MovePossessedTool()
    {
        Transform possessedTransform =
            GetPossessedTransform();

        if (possessedTransform == null)
        {
            return;
        }

        Vector3 position =
            possessedTransform.position;

        position.y +=
            ReadVerticalInput() *
            Mathf.Max(
                0.0f,
                verticalSpeed) *
            Time.deltaTime;

        if (sceneCamera != null &&
            Mouse.current != null)
        {
            Vector2 mousePosition =
                Mouse.current.position.ReadValue();

            Ray ray =
                sceneCamera.ScreenPointToRay(
                    mousePosition);

            Plane movementPlane =
                new Plane(
                    Vector3.up,
                    new Vector3(
                        0.0f,
                        position.y,
                        0.0f));

            if (movementPlane.Raycast(
                    ray,
                    out float distance))
            {
                Vector3 point =
                    ray.GetPoint(
                        distance);

                position.x = point.x;
                position.z = point.z;
            }
        }

        possessedTransform.position =
            position;
    }

    private static float ReadVerticalInput()
    {
        if (Keyboard.current == null)
        {
            return 0.0f;
        }

        float input = 0.0f;

        if (Keyboard.current.wKey.isPressed)
        {
            input += 1.0f;
        }

        if (Keyboard.current.sKey.isPressed)
        {
            input -= 1.0f;
        }

        return input;
    }

    private Transform GetPossessedTransform()
    {
        switch (currentMode)
        {
            case ToolMode.Grasping:
                return graspingTool != null
                    ? graspingTool.transform
                    : null;

            case ToolMode.Cut:
                return cutTool != null
                    ? cutTool.transform
                    : null;

            default:
                return null;
        }
    }
}
