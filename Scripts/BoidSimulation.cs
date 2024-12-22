using System.Threading.Tasks;
using Godot;

namespace Boids.Scripts;

public partial class BoidSimulation : Node
{
    [Export] private float _maxSpeed = 2.0f;
    [Export] private float _maxForce = 0.03f;
    [Export] private float _desiredSeparation = 5.0f;
    [Export] private float _neighborDistance = 20f;
    [Export] private int _flockSize = 100;
    [Export] private float _boundaryThreshold = 10f;
    [Export] private float _boundaryForceFactor = 5f;
    [Export] private Vector3 _minBounds = new(-100f, -25f, -50f);
    [Export] private Vector3 _maxBounds = new(100f, 25f, 50f);
    
    private BoidComponent[] _boids;
    private Node3D[] _boidNodes;

    public override void _Ready()
    {
        _boids = new BoidComponent[_flockSize];
        _boidNodes = new Node3D[_flockSize];
        var boidScene = GD.Load<PackedScene>("res://Boid.tscn");
        for (var i = 0; i < _flockSize; i++)
        {
            _boids[i] = new BoidComponent
            {
                Position = new Vector3(
                    (float)GD.RandRange(_minBounds.X, _maxBounds.X),
                    (float)GD.RandRange(_minBounds.Y, _maxBounds.Y),
                    (float)GD.RandRange(_minBounds.Z, _maxBounds.Z)
                ),
                Velocity = Vector3.Zero
            };
            
            var boidNode = (Node3D)boidScene.Instantiate();
            _boidNodes[i] = boidNode;
            boidNode.Position = _boids[i].Position;
            AddChild(boidNode);
        }
    }
    
    public override void _Process(double delta)
    {
        Simulate((float)delta);
        UpdateNodes();
    }
    
    private void Simulate(float delta)
    {
        Parallel.For(0, _boids.Length, i =>
        {
            _boids[i].Velocity += CalculateVelocities(i) * delta;
            _boids[i].Position += _boids[i].Velocity;
        });
    }
    
    private void UpdateNodes()
    {
        for (var i = 0; i < _boids.Length; i++)
        {
            _boidNodes[i].Position = _boids[i].Position;
            var target = _boids[i].Position - _boids[i].Velocity;
            if(target !=  _boidNodes[i].Position)
                _boidNodes[i].LookAt(target, Vector3.Up);
        }
    }

    private Vector3 CalculateVelocities(int currentBoidIndex)
    {
        var currentBoid = _boids[currentBoidIndex];
        var separationVelocity = Vector3.Zero;
        var separationCount = 0;
        var alignmentVelocity = Vector3.Zero;
        var alignmentCount = 0;
        var cohesionVelocity = Vector3.Zero;
        var cohesionCount = 0;
        for(var j = 0; j < _boids.Length; j++)
        {
            if (currentBoidIndex == j) continue;
            var otherBoid = _boids[j];
            var distance = currentBoid.Position.DistanceTo(otherBoid.Position);
            
            AccumulateVelocity(distance, _desiredSeparation, currentBoid, otherBoid, ref separationVelocity, ref separationCount);
            AccumulateVelocity(distance, _neighborDistance, currentBoid, otherBoid, ref alignmentVelocity, ref alignmentCount);
            AccumulateVelocity(distance, _neighborDistance, currentBoid, otherBoid, ref cohesionVelocity, ref cohesionCount);
        }
        NormalizeVelocity(currentBoid, ref separationVelocity, ref separationCount);
        NormalizeVelocity(currentBoid, ref alignmentVelocity, ref alignmentCount);
        if(cohesionCount > 0)
        {
            cohesionVelocity /= cohesionCount;
            cohesionVelocity = Seek(cohesionVelocity, currentBoid);
        }

        var boundaryVelocity = CalculateBoundaryVelocity(currentBoid);
        return separationVelocity + alignmentVelocity + cohesionVelocity + boundaryVelocity;
    }

    private void AccumulateVelocity(
        float distance,
        float desiredDistance,
        BoidComponent currentBoid, 
        BoidComponent otherBoid, 
        ref Vector3 accumulatedVelocity,
        ref int count
    )
    {
        if (distance > 0 && distance < desiredDistance)
        {
            var diff = (currentBoid.Position - otherBoid.Position).Normalized() / distance;
            accumulatedVelocity += diff;
            count++;
        }
    }

    private void NormalizeVelocity(
        BoidComponent currentBoid, 
        ref Vector3 velocity, 
        ref int separationCount
    )
    {
        if (velocity.Length() > 0)
        {
            velocity /= separationCount;
            velocity = velocity.Normalized() * _maxSpeed;
            velocity -= currentBoid.Velocity;
            velocity = velocity.LimitLength(_maxForce);
        }
    }
    
    private Vector3 Seek(Vector3 target, BoidComponent currentBoidComponent)
    {
        var desired = (target - currentBoidComponent.Position).Normalized() * _maxSpeed;
        var currentVel = currentBoidComponent.Velocity;
        var steer = desired - currentVel;
        steer = steer.LimitLength(_maxForce);
        return steer;
    }
    
    private Vector3 CalculateBoundaryVelocity(BoidComponent boid)
    {
        var desired = Vector3.Zero;
        // Check each axis and see if we're within threshold of a boundary
        // If so, set 'desired' velocity pointing back in
        if (boid.Position.X < _minBounds.X + _boundaryThreshold)
        {
            desired.X = _maxSpeed;
        }
        else if (boid.Position.X > _maxBounds.X - _boundaryThreshold)
        {
            desired.X = -_maxSpeed;
        }

        if (boid.Position.Y < _minBounds.Y + _boundaryThreshold)
        {
            desired.Y = _maxSpeed;
        }
        else if (boid.Position.Y > _maxBounds.Y - _boundaryThreshold)
        {
            desired.Y = -_maxSpeed;
        }

        if (boid.Position.Z < _minBounds.Z + _boundaryThreshold)
        {
            desired.Z = _maxSpeed;
        }
        else if (boid.Position.Z > _maxBounds.Z - _boundaryThreshold)
        {
            desired.Z = -_maxSpeed;
        }

        if (desired == Vector3.Zero)
            return Vector3.Zero;

        // Convert desired direction into a steering force, similar to 'Seek'.
        desired = desired.Normalized() * _maxSpeed;
        var steer = desired - boid.Velocity;
        steer = steer.LimitLength(_maxForce);

        // Scale it by boundaryForceFactor so we can make it gentler if we want
        return steer * _boundaryForceFactor;
    }
}