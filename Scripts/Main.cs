using Godot;

namespace Boids.Scripts;

public partial class Main : Node
{
	[Export] private int _flockSize = 100;
	[Export] private Vector3 _spawnBounds = new(50, 0.5f, 50);
	private Boid[] _flock;

	public override void _Ready()
	{
		_flock = new Boid[_flockSize];
		var boidScene = GD.Load<PackedScene>("res://Boid.tscn");
		for (var i = 0; i < _flockSize; i++)
		{
			var boid = (Boid)boidScene.Instantiate();
			_flock[i] = boid;
			AddChild(boid);
			boid.Position = new Vector3(
				(float)GD.RandRange(-1.0, 1.0) * _spawnBounds.X,
				_spawnBounds.Y,
				(float)GD.RandRange(-1.0, 1.0) * GD.Randf() * _spawnBounds.Z
			);
			boid.Velocity = Vector3.Zero;
		}
	}
	
	public override void _Process(double delta)
	{
		foreach (var boid in _flock)
		{
			boid.Flock(_flock);
		}
	}
}