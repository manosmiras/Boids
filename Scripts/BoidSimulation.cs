using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace Boids.Scripts;

// TODO: Spatial partitioning
public partial class BoidSimulation : Node
{
    [Export] public float MinSeparationDistance = 10f;
    [Export] public float SeparationFactor = 0.05f;
    [Export] public float AlignmentFactor = 0.05f;
    [Export] public float CohesionFactor = 0.05f;
    [Export] public float VisualRange = 25f;
    [Export] public float MaxSpeed = 0.4f;
    [Export] public Vector3 Bounds = new(25f, 25f, 25f);
    
    [Export] private int _flockSize = 1000;
    [Export] private float _boundaryThreshold = 0f;
    
    private Boid[] _boids;
    private MultiMeshInstance3D _multiMeshInstance;
    private MultiMesh _multiMesh;
    private Octree _octree;

    public override void _Ready()
    {
        _octree = new Octree(Vector3.Zero, Bounds, 20);
        _boids = new Boid[_flockSize];
        for (var i = 0; i < _flockSize; i++)
        {
            var position = new Vector3(
                (float)GD.RandRange(-Bounds.X * .3f, Bounds.X * .3f),
                (float)GD.RandRange(-Bounds.Y * .3f, Bounds.Y * .3f),
                (float)GD.RandRange(-Bounds.Z * .3f, Bounds.Z * .3f)
            );
            _boids[i] = new Boid
            {
                Position = position,
                Velocity = new Vector3(
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed),
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed),
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed)
                )
            };
            _octree.Insert(point: position);
        }
        var boidScene = GD.Load<PackedScene>("res://Boid.tscn");
        _multiMeshInstance = boidScene.Instantiate<MultiMeshInstance3D>();
        _multiMesh = _multiMeshInstance.Multimesh;
        _multiMesh.TransformFormat = MultiMesh.TransformFormatEnum.Transform3D;
        _multiMesh.InstanceCount = _flockSize;
        AddChild(_multiMeshInstance);
        for (var i = 0; i < _flockSize; i++)
        {
            var transform = Transform3D.Identity;
            transform.Origin = _boids[i].Position;
            _multiMesh.SetInstanceTransform(i, transform);
        }
    }
    
    public override void _Process(double delta)
    {
        Simulate((float)delta);
        UpdateVisuals();
    }
    
    private void Simulate(float delta)
    {
        Parallel.For(0, _boids.Length, i =>
        {
            _boids[i].Velocity += CalculateVelocities(i) * delta;
            var oldPosition = _boids[i].Position;
            var newPosition = _boids[i].Position + _boids[i].Velocity;
            _octree.Update(oldPosition, newPosition);
            _boids[i].Position = newPosition;
        });
    }

    private Vector3 CalculateVelocities(int i)
    {
        var boidsInVisualRange = _octree.Query(_boids[i].Position, Vector3.One * VisualRange * 0.5f);
        var boidsInMinSeparationRange = _octree.Query(_boids[i].Position, Vector3.One * MinSeparationDistance * 0.5f);
        //GD.Print($"Nearby boid count: {nearbyBoidPositions.Count}");
        var separation = AvoidOthers(i, boidsInMinSeparationRange);
        //var alignment = MatchVelocity(i, boidsInVisualRange);
        var cohesion = GoTowardsCenter(i, boidsInVisualRange);
        var bounds = KeepWithinBounds(i);
        return LimitSpeed(separation + /*alignment +*/ cohesion + bounds);
    }
    
    private Vector3 AvoidOthers(int i, List<Vector3> nearbyBoidPositions)
    {
        var velocity = Vector3.Zero;
        foreach (var nearbyBoidPosition in nearbyBoidPositions)
        {
            var diff = _boids[i].Position - nearbyBoidPosition;
            velocity += diff;
        }
        
        /*for (var j = 0; j < _boids.Length; j++)
        {
            if(i == j) continue;
            var distance = _boids[i].Position.DistanceTo(_boids[j].Position);
            if (distance < MinSeparationDistance)
            {
                var diff = _boids[i].Position - _boids[j].Position;
                velocity += diff;
            }
        }*/
        return velocity * SeparationFactor;
    }
    
    private Vector3 MatchVelocity(int i, List<Vector3> nearbyBoidPositions)
    {
        var velocity = Vector3.Zero;
        var neighbourCount = 0;
        
        // TODO: Make the octree take in a generic type so I can pass the boid itself
        /*foreach (var nearbyBoidPosition in nearbyBoidPositions)
        {
            var diff = _boids[i].Position - nearbyBoidPosition;
            velocity += diff;
        }*/
        
        for (var j = 0; j < _boids.Length; j++)
        {
            if(i == j) continue;
            var distance = _boids[i].Position.DistanceTo(_boids[j].Position);
            if (distance < VisualRange)
            {
                velocity += _boids[j].Velocity;
                neighbourCount++;
            }
        }
        if (neighbourCount == 0)
            return Vector3.Zero;
        velocity /= neighbourCount;
        return (velocity - _boids[i].Velocity) * AlignmentFactor;
    }
    
    private Vector3 GoTowardsCenter(int i, List<Vector3> nearbyBoidPositions)
    {
        var velocity = Vector3.Zero;
        var neighbourCount = 0;
        
        foreach (var nearbyBoidPosition in nearbyBoidPositions)
        {
            velocity += nearbyBoidPosition;
            neighbourCount++;
        }
        
        /*for (var j = 0; j < _boids.Length; j++)
        {
            if(i == j) continue;
            var distance = _boids[i].Position.DistanceTo(_boids[j].Position);
            if (distance < VisualRange)
            {
                velocity += _boids[j].Position;
                neighbourCount++;
            }
        }*/
        if (neighbourCount == 0)
            return Vector3.Zero;
        velocity /= neighbourCount;
        var desired = velocity - _boids[i].Position;
        return desired.Normalized() * MaxSpeed;
    }
    
    private Vector3 LimitSpeed(Vector3 velocity)
    {
        if (velocity.Length() > MaxSpeed)
        {
            velocity = velocity.Normalized() * MaxSpeed;
        }
        return velocity;
    }
    
    
    private Vector3 KeepWithinBounds(int i)
    {
        var boid = _boids[i];
        var velocity = Vector3.Zero;
        var boundForce = MaxSpeed;
        if(boid.Position.X < -Bounds.X * .5f + _boundaryThreshold)
            velocity.X += 1;
        if(boid.Position.X > Bounds.X * .5f - _boundaryThreshold)
            velocity.X -= 1;
        if(boid.Position.Y < -Bounds.Y * .5f + _boundaryThreshold)
            velocity.Y += 1;
        if(boid.Position.Y > Bounds.Y * .5f - _boundaryThreshold)
            velocity.Y -= 1;
        if(boid.Position.Z < -Bounds.Z * .5f + _boundaryThreshold)
            velocity.Z += 1;
        if(boid.Position.Z > Bounds.Z * .5f - _boundaryThreshold)
            velocity.Z -= 1;
        return velocity.Normalized() * boundForce;
    }
    
    private void UpdateVisuals()
    {
        for (var i = 0; i < _flockSize; i++)
        {
            var currentBoid = _boids[i];
            var transform = Transform3D.Identity;
            transform.Origin = currentBoid.Position;
            if (currentBoid.Velocity.Length() > 0)
            {
                var forward = currentBoid.Velocity.Normalized();
                var basis = BuildLookAtBasis(forward, Vector3.Up);
                transform.Basis = basis;
            }
            _multiMesh.SetInstanceTransform(i, transform);
        }
    }
    
    private Basis BuildLookAtBasis(Vector3 forward, Vector3 up)
    {
        forward = forward.Normalized();
        var xAxis = up.Cross(forward).Normalized();
        var yAxis = forward.Cross(xAxis).Normalized();
        return new Basis(xAxis, yAxis, forward);
    }
}