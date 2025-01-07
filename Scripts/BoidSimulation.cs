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
    [Export] public float MaxSpeed = 0.1f;
    [Export] public Vector3 SimulationBounds = new(10000f, 10000f, 10000f);
    [Export] public Vector3 SpawnBounds = new(50f, 50f, 50f);
    
    [Export] private int _flockSize = 1000;
    [Export] private float _boundaryThreshold = 100f;
    
    private Boid[] _boids;
    private MultiMeshInstance3D _multiMeshInstance;
    private MultiMesh _multiMesh;
    private Octree<Boid> _octree;

    public override void _Ready()
    {
        _octree = new Octree<Boid>(Vector3.Zero, SimulationBounds, 20);
        _boids = new Boid[_flockSize];
        for (ushort i = 0; i < _flockSize; i++)
        {
            var position = new Vector3(
                (float)GD.RandRange(-SpawnBounds.X * .5f, SpawnBounds.X * .5f),
                (float)GD.RandRange(-SpawnBounds.Y * .5f, SpawnBounds.Y * .5f),
                (float)GD.RandRange(-SpawnBounds.Z * .5f, SpawnBounds.Z * .5f)
            );
            _boids[i] = new Boid
            {
                Id = i,
                Position = position,
                Velocity = new Vector3(
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed),
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed),
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed)
                )
            };
            _octree.Insert(_boids[i]);
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
            var previous = _boids[i];
            var updated = previous with {Position = previous.Position + previous.Velocity};
            //if (IsWithinBounds(updated.Position))
            //{
                _octree.Update(previous, updated);
                _boids[i] = updated;
            //}
        });
    }

    private Vector3 CalculateVelocities(int i)
    {
        var boidsInVisualRange = _octree.Query(_boids[i].Position, Vector3.One * VisualRange * 0.5f);
        var boidsInMinSeparationRange = _octree.Query(_boids[i].Position, Vector3.One * MinSeparationDistance * 0.5f);
        var separation = AvoidOthers(i, boidsInMinSeparationRange);
        var alignment = MatchVelocity(i, boidsInVisualRange);
        var cohesion = GoTowardsCenter(i, boidsInVisualRange);
        var velocity = LimitSpeed(separation + alignment + cohesion);
        return KeepWithinBounds(i, velocity);
    }
    
    private Vector3 AvoidOthers(int i, List<Boid> nearbyBoids)
    {
        var velocity = Vector3.Zero;
        foreach (var nearbyBoid in nearbyBoids)
        {
            if(_boids[i].Id == nearbyBoid.Id) continue;
            var diff = _boids[i].Position - nearbyBoid.Position;
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
    
    private Vector3 MatchVelocity(int i, List<Boid> nearbyBoids)
    {
        var velocity = Vector3.Zero;
        var neighbourCount = 0;
        
        // TODO: Make the octree take in a generic type so I can pass the boid itself
        foreach (var nearbyBoid in nearbyBoids)
        {
            if(_boids[i].Id == nearbyBoid.Id) continue;
            velocity += nearbyBoid.Velocity;
            neighbourCount++;
        }
        
        /*for (var j = 0; j < _boids.Length; j++)
        {
            if(i == j) continue;
            var distance = _boids[i].Position.DistanceTo(_boids[j].Position);
            if (distance < VisualRange)
            {
                velocity += _boids[j].Velocity;
                neighbourCount++;
            }
        }*/
        if (neighbourCount == 0)
            return Vector3.Zero;
        velocity /= neighbourCount;
        return (velocity - _boids[i].Velocity) * AlignmentFactor;
    }
    
    private Vector3 GoTowardsCenter(int i, List<Boid> nearbyBoids)
    {
        var velocity = Vector3.Zero;
        var neighbourCount = 0;
        
        foreach (var nearbyBoid in nearbyBoids)
        {
            if(_boids[i].Id == nearbyBoid.Id) continue;
            velocity += nearbyBoid.Position;
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
    
    
    private Vector3 KeepWithinBounds(int i, Vector3 newVelocity)
    {
        var newPosition = _boids[i].Position + newVelocity;
        if (newPosition.X < -SimulationBounds.X * .5f + _boundaryThreshold || newPosition.X > SimulationBounds.X * .5f - _boundaryThreshold)
            newVelocity.X = -newVelocity.X;

        if (newPosition.Y < -SimulationBounds.Y * .5f + _boundaryThreshold || newPosition.Y > SimulationBounds.Y * .5f - _boundaryThreshold)
            newVelocity.Y = -newVelocity.Y;

        if (newPosition.Z < -SimulationBounds.Z * .5f + _boundaryThreshold || newPosition.Z > SimulationBounds.Z * .5f - _boundaryThreshold)
            newVelocity.Z = -newVelocity.Z;
        
        return newVelocity;
    }

    private bool IsWithinBounds(Vector3 position)
    {
        return position.X > -SimulationBounds.X * .5f && position.X < SimulationBounds.X * .5f &&
               position.Y > -SimulationBounds.Y * .5f && position.Y < SimulationBounds.Y * .5f &&
               position.Z > -SimulationBounds.Z * .5f && position.Z < SimulationBounds.Z * .5f;
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