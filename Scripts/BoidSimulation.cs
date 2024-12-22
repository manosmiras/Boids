using System.Threading.Tasks;
using Godot;

namespace Boids.Scripts;

public partial class BoidSimulation : Node
{
    public BoidComponent[] Boids;
    [Export] private float _maxSpeed = 2.0f;
    [Export] private float _maxForce = 0.03f;
    [Export] private float _desiredSeparation = 25.0f;
    [Export] private float _neighborDistance = 50f;
    [Export] private float _turnFactor = 20.0f;
    
    [Export] private int _flockSize = 100;
    private Vector3 _minBounds = new(-25f, -25f, -25f);
    private Vector3 _maxBounds = new(25f, 25f, 25f);
    private Node3D[] _boidNodes;

    public override void _Ready()
    {
        Boids = new BoidComponent[_flockSize];
        _boidNodes = new Node3D[_flockSize];
        var boidScene = GD.Load<PackedScene>("res://Boid.tscn");
        for (var i = 0; i < _flockSize; i++)
        {
            Boids[i] = new BoidComponent
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
            boidNode.Position = Boids[i].Position;
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
        Parallel.For(0, Boids.Length, i =>
        {
            Boids[i].Velocity += CalculateVelocities(i) * delta;
            Boids[i].Position += Boids[i].Velocity;
        });
    }
    
    private void UpdateNodes()
    {
        for (var i = 0; i < Boids.Length; i++)
        {
            _boidNodes[i].Position = Boids[i].Position;
            var movingTowards = Boids[i].Position - Boids[i].Velocity;
            if(movingTowards != Vector3.Zero)
                _boidNodes[i].LookAt(movingTowards, Vector3.Up);
        }
    }

    private Vector3 CalculateVelocities(int currentBoidIndex)
    {
        var currentBoid = Boids[currentBoidIndex];
        var separationVelocity = Vector3.Zero;
        var separationCount = 0;
        var alignmentVelocity = Vector3.Zero;
        var alignmentCount = 0;
        var cohesionVelocity = Vector3.Zero;
        var cohesionCount = 0;
        for(var j = 0; j < Boids.Length; j++)
        {
            if (currentBoidIndex == j) continue;
            var otherBoid = Boids[j];
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
        return separationVelocity + alignmentVelocity + cohesionVelocity;
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
}