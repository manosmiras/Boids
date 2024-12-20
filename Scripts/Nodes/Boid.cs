using Godot;

namespace Boids.Scripts
{
    
    public partial class Boid : Node3D
    {
        public BoidComponent[] Boids;
        public int Index;

        public override void _Process(double delta)
        {
            Position = Boids[Index].Position;
            // Look forward
            var lookAt = Boids[Index].Position - Boids[Index].Velocity;
            LookAt(lookAt, Vector3.Up);
        }
    }
}
