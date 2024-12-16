using System.Collections.Generic;
using Godot;

namespace Boids.Scripts;

public partial class Boid : CharacterBody3D
{
	public void Flock(List<Boid> flock)
	{
		SetVelocity(new Vector3(1.0f, 0.0f, 1.0f));
		MoveAndSlide();
	}
}