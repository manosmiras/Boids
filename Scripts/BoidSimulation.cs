using System.Threading.Tasks;
using Godot;

namespace Boids.Scripts;

// TODO: Spatial partitioning
public partial class BoidSimulation : Node
{
    [Export] public float MinSeparationDistance = 50f;
    [Export] public float SeparationFactor = 0.05f;
    [Export] public float AlignmentFactor = 0.05f;
    [Export] public float CohesionFactor = 0.05f;
    [Export] public float VisualRange = 100f;
    [Export] public float MaxSpeed = 0.4f;
    [Export] public Vector3 Bounds = new(25f, 25f, 25f);
    
    [Export] private int _flockSize = 100;
    [Export] private float _boundaryThreshold = 0f;
    
    private Boid[] _boids;
    private MultiMeshInstance3D _multiMeshInstance;
    private MultiMesh _multiMesh;

    public override void _Ready()
    {
        _boids = new Boid[_flockSize];
        for (var i = 0; i < _flockSize; i++)
        {
            _boids[i] = new Boid
            {
                Position = new Vector3(
                    (float)GD.RandRange(-Bounds.X, Bounds.X),
                    (float)GD.RandRange(-Bounds.Y, Bounds.Y),
                    (float)GD.RandRange(-Bounds.Z, Bounds.Z)
                ),
                Velocity = new Vector3(
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed),
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed),
                    (float)GD.RandRange(-MaxSpeed, MaxSpeed)
                )
            };
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
            _boids[i].Position += _boids[i].Velocity;
        });
    }

    private Vector3 CalculateVelocities(int i)
    {
        var separation = AvoidOthers(i);
        var alignment = MatchVelocity(i);
        var cohesion = GoTowardsCenter(i);
        var bounds = KeepWithinBounds(i);
        return LimitSpeed(separation + alignment + cohesion + bounds);
    }
    
    private Vector3 AvoidOthers(int i)
    {
        var velocity = Vector3.Zero;
        for (var j = 0; j < _boids.Length; j++)
        {
            if(i == j) continue;
            var distance = _boids[i].Position.DistanceTo(_boids[j].Position);
            if (distance < MinSeparationDistance)
            {
                var diff = _boids[i].Position - _boids[j].Position;
                velocity += diff;
            }
        }
        return velocity * SeparationFactor;
    }
    
    private Vector3 MatchVelocity(int i)
    {
        var velocity = Vector3.Zero;
        var neighbourCount = 0;
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
    
    private Vector3 GoTowardsCenter(int i)
    {
        var velocity = Vector3.Zero;
        var neighbourCount = 0;
        for (var j = 0; j < _boids.Length; j++)
        {
            if(i == j) continue;
            var distance = _boids[i].Position.DistanceTo(_boids[j].Position);
            if (distance < VisualRange)
            {
                velocity += _boids[j].Position;
                neighbourCount++;
            }
        }
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
        if(boid.Position.X < -Bounds.X + _boundaryThreshold)
            velocity.X += 1;
        if(boid.Position.X > Bounds.X - _boundaryThreshold)
            velocity.X -= 1;
        if(boid.Position.Y < -Bounds.Y + _boundaryThreshold)
            velocity.Y += 1;
        if(boid.Position.Y > Bounds.Y - _boundaryThreshold)
            velocity.Y -= 1;
        if(boid.Position.Z < -Bounds.Z + _boundaryThreshold)
            velocity.Z += 1;
        if(boid.Position.Z > Bounds.Z - _boundaryThreshold)
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