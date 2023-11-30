﻿using BepuPhysicIntegrationTest.Integration.Extensions;
using Stride.Engine;
using Stride.Input;

namespace BepuPhysicIntegrationTest.Integration.Components.Utils
{
    //[DataContract("SpawnerComponent", Inherited = true)]
    [ComponentCategory("Bepu - Utils")]
    public class ThrowerComponent : Spawner
    {
        public Entity? SpawnPosition { get; set; }

        public float Speed { get; set; } = 20f;

        public override void SimulationUpdate(float timeStep)
        {
            if (SpawnPosition == null) return;

            if (Input.IsKeyPressed(Keys.T))
            {
                var camera = Game.Services.GetService<SceneSystem>().GraphicsCompositor.Cameras[0].Camera;
                var forward = Stride.Core.Mathematics.Vector3.TransformNormal(-Stride.Core.Mathematics.Vector3.UnitZ, Stride.Core.Mathematics.Matrix.RotationQuaternion(camera.Entity.Transform.Rotation)).ToNumericVector();

                Spawn(SpawnPosition.Transform.GetWorldPos(), (forward * Speed).ToStrideVector(), new());
            }
        }

        public override void Update()
        {
            DebugText.Print("Throw a prefab (T)", new(BepuAndStrideExtensions.X_DEBUG_TEXT_POS, 125));
        }
    }
}
