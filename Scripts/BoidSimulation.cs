using Godot;

namespace Boids.Scripts;

public class BoidSimulation
{
    public BoidComponent[] Boids;
    public float MaxSpeed = 12.0f;
    public float MaxForce = 12.0f;
    public float DesiredSeparation = 25f;
    public float NeighborDistance = 50f;
    
    private float DesiredSeparationSquared => DesiredSeparation * DesiredSeparation;
    private float NeighborDistanceSquared => NeighborDistance * NeighborDistance;


    public BoidSimulation(int count, Vector3 spawnBounds)
    {
        Boids = new BoidComponent[count];
        for (var i = 0; i < count; i++)
        {
            Boids[i] = new BoidComponent
            {
                Position = new Vector3(
                    (float)GD.RandRange(-1.0, 1.0) * spawnBounds.X,
                    spawnBounds.Y,
                    (float)GD.RandRange(-1.0, 1.0) * GD.Randf() * spawnBounds.Z
                ),
                Velocity = Vector3.Zero
            };
        }
    }
    
    public void Simulate(float delta)
    {
        for (var i = 0; i < Boids.Length; i++)
        {
            var separationVelocity = Separate(i);
            var alignmentVelocity = Align(i);
            var cohesionVelocity = Cohere(i);
            Boids[i].Velocity += (separationVelocity + alignmentVelocity + cohesionVelocity) * delta;
            Boids[i].Position += Boids[i].Velocity * delta;
        }
    }
    
    private Vector3 Separate(int currentBoidIndex)
    {
        var currentBoid = Boids[currentBoidIndex];
        var separationVelocity = Vector3.Zero;
        var separationCount = 0;
        for(var j = 0; j < Boids.Length; j++)
        {
            if (currentBoidIndex == j) continue;
            var otherBoid = Boids[j];
            var dist = currentBoid.Position.DistanceSquaredTo(otherBoid.Position);
            if (dist > 0 && dist < DesiredSeparationSquared)
            {
                var diff = (currentBoid.Position - otherBoid.Position).Normalized() / dist;
                separationVelocity += diff;
                separationCount++;
            }
        }
        
        if(separationCount > 0)
        {
            separationVelocity /= separationCount;
        }
            
        if (separationVelocity.Length() > 0)
        {
            separationVelocity = separationVelocity.Normalized() * MaxSpeed;
            separationVelocity -= currentBoid.Velocity;
            separationVelocity = separationVelocity.LimitLength(MaxForce);
        }

        return separationVelocity;
    }

    private Vector3 Align(int currentBoidIndex)
    {
        var currentBoid = Boids[currentBoidIndex];
        var alignmentVelocity = Vector3.Zero;
        var alignmentCount = 0;
        for (var j = 0; j < Boids.Length; j++)
        {
            if (currentBoidIndex == j) continue;
            var otherBoid = Boids[j];
            var dist = currentBoid.Position.DistanceSquaredTo(otherBoid.Position);
            if (dist > 0 && dist < NeighborDistanceSquared)
            {
                alignmentVelocity += otherBoid.Velocity;
                alignmentCount++;
            }
        }
        
        if (alignmentCount > 0)
        {
            alignmentVelocity /= alignmentCount;
            alignmentVelocity = alignmentVelocity.Normalized() * MaxSpeed;
            alignmentVelocity -= currentBoid.Velocity;
            alignmentVelocity = alignmentVelocity.LimitLength(MaxForce);
        }

        return alignmentVelocity;
    }
    
    private Vector3 Cohere(int currentBoidIndex)
    {
        var currentBoid = Boids[currentBoidIndex];
        var cohereVelocity = Vector3.Zero;
        var cohereCount = 0;
        for (var j = 0; j < Boids.Length; j++)
        {
            if (currentBoidIndex == j) continue;
            var otherBoid = Boids[j];
            var dist = currentBoid.Position.DistanceSquaredTo(otherBoid.Position);
            if (dist > 0 && dist < NeighborDistanceSquared)
            {
                cohereVelocity += otherBoid.Position;
                cohereCount++;
            }
        }
        
        if (cohereCount > 0)
        {
            cohereVelocity /= cohereCount;
            cohereVelocity = Seek(cohereVelocity, currentBoid);
        }

        return cohereVelocity;
    }
    
    private Vector3 Seek(Vector3 target, BoidComponent currentBoidComponent)
    {
        var desired = (target - currentBoidComponent.Position).Normalized() * MaxSpeed;
        var currentVel = currentBoidComponent.Velocity;
        var steer = desired - currentVel;
        steer = steer.LimitLength(MaxForce);
        return steer;
    }
}