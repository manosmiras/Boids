using Godot;

namespace Boids.Scripts;

public struct Boid : IPosition
{
    public ushort Id
    {
        get;
        set;
    }
    public Vector3 Position
    {
        get;
        set;
    }
    public Vector3 Velocity
    {
        get;
        set;
    }
}