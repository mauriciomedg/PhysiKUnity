using UnityEngine;

namespace PhysiK.Unity
{
    [CreateAssetMenu(
        fileName = "PhysikMaterial",
        menuName = "PhysiK/Material")]
    public sealed class PhysikMaterialAsset : ScriptableObject
    {
        [Min(0.0f)]
        public float density = 1.0f;

        [Min(0.0f)]
        public float youngModulus = 25.0f;

        [Range(0.0f, 0.49f)]
        public float poissonRatio = 0.3f;

        [Min(0.0f)]
        public float damping = 0.25f;

        public PhysikMaterialDesc ToNative()
        {
            return new PhysikMaterialDesc
            {
                density = density,
                youngModulus = youngModulus,
                poissonRatio = poissonRatio,
                damping = damping
            };
        }
    }
}
