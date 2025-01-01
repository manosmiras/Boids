using System.Collections.Generic;
using Godot;

namespace Boids.Scripts;

public partial class OctreeVisualizer : Node3D
{
    [Export] private Node3D _root;
    [Export] private Vector3 _size = new(25f, 25f, 25f);
    [Export] private int _capacity = 5;
    [Export] private int _pointCount = 40;
    private Octree _octree;
    private List<Node3D> _pointVisuals = new();
    private List<Node3D> _boundVisuals = new();
    
    public override void _Ready()
    {
        _octree = new Octree(Vector3.Zero, _size, _capacity);
        
        for (var i = 0; i < _pointCount; i++)
        {
            var point = new Vector3(
                (float)GD.RandRange(-_size.X * 0.3f, _size.X * 0.3f),
                (float)GD.RandRange(-_size.Y * 0.3f, _size.Y * 0.3f),
                (float)GD.RandRange(-_size.Z * 0.3f, _size.Z * 0.3f)
            );
            SpawnPointVisualization(point);
            _octree.Insert(point);
        }
		
        var nodeCount = _octree.CountNodes();
        var pointCount = _octree.CountPoints();
        GD.Print($"Node count: {nodeCount}, Point count: {pointCount}");
    }
    
    public override void _Process(double delta)
    {
        if (_octree == null)
            return;
        var oldPositions = new List<Vector3>();
        foreach (var sphere in _pointVisuals)
        {
            oldPositions.Add(sphere.GlobalPosition);
        }
        _root.Rotate(Vector3.Up, 1 * (float)delta);
        for (var i = 0; i < _pointVisuals.Count; i++)
        {
           _octree.Update(oldPositions[i], _pointVisuals[i].GlobalPosition);
        }
        
        UpdateBoundVisuals();
    }

    private void UpdateBoundVisuals()
    {
        foreach (var visual in _boundVisuals)
        {
            RemoveChild(visual);
        }
        _boundVisuals.Clear();
        var bounds = _octree.GetAllBounds();
        foreach (var bound in bounds)
        {
            SpawnBoundsVisualization(bound.GetCenter(), bound.Size);
        }
    }
    
    private void SpawnPointVisualization(Vector3 position)
    {
        var sphere = new MeshInstance3D();
        sphere.Mesh = new SphereMesh();
        var material = new StandardMaterial3D
        {
            AlbedoColor = new Color(1, 0, 0)
        };
        sphere.Mesh.SurfaceSetMaterial(0, material);
		
        sphere.Transform = new Transform3D(Basis.Identity, position);
        _root.AddChild(sphere);
        _pointVisuals.Add(sphere);
    }
    
    private void SpawnBoundsVisualization(Vector3 position, Vector3 scale)
    {
        var cube = new MeshInstance3D();
        cube.Mesh = new BoxMesh();
        var material = new StandardMaterial3D
        {
            AlbedoColor = new Color(1, 0, 0, 0.1f),
            Transparency = BaseMaterial3D.TransparencyEnum.Alpha
        };
        cube.Mesh.SurfaceSetMaterial(0, material);
        cube.Transform = new Transform3D(Basis.Identity, position);
        cube.Scale = scale;
        AddChild(cube);
        _boundVisuals.Add(cube);
    }
}