using System.Collections.Generic;
using Godot;

namespace Boids.Scripts;

public partial class Main : Node
{
	[Export] private int _flockSize = 100;
	[Export] private Vector3 _spawnBounds = new Vector3(50, 1, 50);
	private List<Boid> _flock = new();
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		var boidScene = GD.Load<PackedScene>("res://Boid.tscn");
		for (var i = 0; i < _flockSize; i++)
		{
			var boid = (Boid)boidScene.Instantiate();
			_flock.Add(boid);
			AddChild(boid);
			boid.Position = new Vector3(
				(float)GD.RandRange(-1.0, 1.0) * _spawnBounds.X,
				_spawnBounds.Y,
				(float)GD.RandRange(-1.0, 1.0) * GD.Randf() * _spawnBounds.Z
			);
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		foreach (var boid in _flock)
		{
			boid.Flock(_flock);
		}
	}
}