using Godot;

namespace Boids.Scripts
{
    // TODO: Create a struct per boid, pass an array of boid structs to the flock method so we're not passing about a lot of CharacterBody3Ds
    // TODO: Spatial partitioning (Quadtree / Octtree) to reduce the number of boids we need to check against
    // TODO: Multi-threading
    public partial class Boid : CharacterBody3D
    {
        public float MaxSpeed = 12.0f;
        public float MaxForce = 12.0f;
        public float DesiredSeparation = 25f;
        public float NeighborDistance = 50f;
        public float Gravity = -9.8f;
        
        private float DesiredSeparationSquared => DesiredSeparation * DesiredSeparation;
        private float NeighborDistanceSquared => NeighborDistance * NeighborDistance;
        
        private static Vector2 To2D(Vector3 v) => new(v.X, v.Z);
        private static Vector3 To3D(Vector2 v) => new(v.X, 0f, v.Y);

        public void Flock(Boid[] flock)
        {
            var pos2D = To2D(GlobalPosition);
            
            var separate = Separate(flock, pos2D);
            var align = Align(flock, pos2D);
            var cohesion = Cohesion(flock, pos2D);
            
            var accel3D = To3D(separate + align + cohesion);
            var delta = (float)GetProcessDeltaTime();
            
            if (!IsOnFloor())
            {
                accel3D.Y += Gravity;
            }
            Velocity += accel3D * delta;
            MoveAndSlide();
            if (Velocity.Length() > 0.001f)
            {
                LookAt(GlobalPosition - Velocity, Vector3.Up);
            }
        }

        private Vector2 Separate(Boid[] flock, Vector2 pos2D)
        {
            var steer = Vector2.Zero;
            var count = 0;

            foreach (var boid in flock)
            {
                var otherPos2D = To2D(boid.GlobalPosition);
                var dist = pos2D.DistanceSquaredTo(otherPos2D);
                if (dist > 0 && dist < DesiredSeparationSquared)
                {
                    var diff = (pos2D - otherPos2D).Normalized() / dist;
                    steer += diff;
                    count++;
                }
            }

            if (count > 0)
            {
                steer /= count;
            }

            if (steer.Length() > 0)
            {
                steer = steer.Normalized() * MaxSpeed;
                var currentVel = To2D(GetVelocity());
                steer -= currentVel;
                steer = steer.LimitLength(MaxForce);
            }

            return steer;
        }

        private Vector2 Align(Boid[] flock, Vector2 pos2D)
        {
            var sum = Vector2.Zero;
            var count = 0;

            foreach (var boid in flock)
            {
                var otherPos2D = To2D(boid.GlobalPosition);
                var dist = pos2D.DistanceSquaredTo(otherPos2D);
                if (dist > 0 && dist < NeighborDistanceSquared)
                {
                    sum += To2D(boid.GetVelocity());
                    count++;
                }
            }

            if (count > 0)
            {
                sum /= count;
                sum = sum.Normalized() * MaxSpeed;
                var currentVel = To2D(GetVelocity());
                var steer = sum - currentVel;
                steer = steer.LimitLength(MaxForce);
                return steer;
            }

            return Vector2.Zero;
        }

        private Vector2 Cohesion(Boid[] flock, Vector2 pos2D)
        {
            var sum = Vector2.Zero;
            var count = 0;

            foreach (var boid in flock)
            {
                var otherPos2D = To2D(boid.GlobalPosition);
                var dist = pos2D.DistanceSquaredTo(otherPos2D);
                if (dist > 0 && dist < NeighborDistanceSquared)
                {
                    sum += otherPos2D;
                    count++;
                }
            }

            if (count > 0)
            {
                sum /= count;
                return Seek(sum, pos2D);
            }

            return Vector2.Zero;
        }

        private Vector2 Seek(Vector2 target2D, Vector2 pos2D)
        {
            var desired = (target2D - pos2D).Normalized() * MaxSpeed;
            var currentVel = To2D(GetVelocity());
            var steer = desired - currentVel;
            steer = steer.LimitLength(MaxForce);
            return steer;
        }
    }
}
