using Godot;

namespace Boids.Scripts;

public partial class Main : Node
{
	[Export] private int _flockSize = 100;
	[Export] private Vector3 _spawnBounds = new(50, 0.5f, 50);
	private Boid[] _boids;
	private BoidSimulation _boidSimulation;

	public override void _Ready()
	{
		_boidSimulation = new BoidSimulation(_flockSize, _spawnBounds);
		_boids = new Boid[_flockSize];
		var boidScene = GD.Load<PackedScene>("res://Boid.tscn");
		for (var i = 0; i < _flockSize; i++)
		{
			var boid = (Boid)boidScene.Instantiate();
			boid.Index = i;
			_boids[i] = boid;
			boid.Boids = _boidSimulation.Boids;
			boid.Position = _boidSimulation.Boids[i].Position;
			AddChild(boid);
		}
	}
	
	public override void _Process(double delta)
	{
		_boidSimulation.Simulate((float)delta);
		for (var i = 0; i < _flockSize; i++)
		{
			_boids[i].Position = _boidSimulation.Boids[i].Position;
		}
	}
}