using GdUnit4;
using Godot;
using static GdUnit4.Assertions;

namespace Boids.Scripts.Tests;

[TestSuite]
public class OctreeTest
{
    [TestCase]
    public void TestSubdivideHasCorrectNodeCount()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(0, 0, 0));
        octree.Insert(new Vector3(5, 5, 5));
        octree.Insert(new Vector3(10, 10, 10));
        var nodeCount = octree.CountNodes();

        AssertInt(nodeCount).IsEqual(9);
    }
    
    [TestCase]
    public void TestSubdivideHasCorrectPointCount()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(0, 0, 0));
        octree.Insert(new Vector3(5, 5, 5));
        octree.Insert(new Vector3(-5, -5, -5));
        var pointCount = octree.CountPoints();

        AssertInt(pointCount).IsEqual(3);
    }
    
    [TestCase]
    public void TestSubdivideHasCorrectBounds()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(0, 0, 0));
        octree.Insert(new Vector3(5, 5, 5));
        octree.Insert(new Vector3(-5, -5, -5));
        var bounds = octree.Bounds;
        AssertVector(bounds.GetCenter()).IsEqual(new Vector3(0, 0, 0));
        AssertVector(bounds.Position).IsEqual(new Vector3(-5, -5, -5));
        AssertVector(bounds.Size).IsEqual(new Vector3(10, 10, 10));
        var childBounds = octree.Children![0].Bounds;
        AssertVector(childBounds.GetCenter()).IsEqual(new Vector3(-2.5f, -2.5f, -2.5f));
        AssertVector(childBounds.Position).IsEqual(new Vector3(-5, -5, -5));
        AssertVector(childBounds.Size).IsEqual(new Vector3(5, 5, 5));
    }
    
    [TestCase]
    public void TestInsertHasCorrectPosition()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(0, 0, 0));
        var point = octree.Points[0];
        AssertVector(point).IsEqual(new Vector3(0, 0, 0));
    }

    [TestCase]
    public void TestInsertHasCorrectBounds()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(0, 0, 0));
        var bounds = octree.Bounds;
        AssertVector(bounds.GetCenter()).IsEqual(new Vector3(0, 0, 0));
        AssertVector(bounds.Position).IsEqual(new Vector3(-5, -5, -5));
        AssertVector(bounds.Size).IsEqual(new Vector3(10, 10, 10));
    }
    
    [TestCase]
    public void TestQueryReturnsCorrectPoints()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(-2.5f, -2.5f, -2.5f));
        octree.Insert(new Vector3(2.5f, 3f, 2.5f));
        octree.Insert(new Vector3(2.5f, 4f, 2.5f));
        var points = octree.Query(new Vector3(2.5f, 3.5f, 2.5f), Vector3.One);
        AssertInt(points.Count).IsEqual(2);
        AssertVector(points[0]).IsEqual(new Vector3(2.5f, 3f, 2.5f));
        AssertVector(points[1]).IsEqual(new Vector3(2.5f, 4f, 2.5f));
    }
    
    [TestCase]
    public void TestUpdateUpdatesPosition()
    {
        var octree = new Octree(Vector3.Zero, new Vector3(10, 10, 10), 2);
        octree.Insert(new Vector3(0, 0, 0));
        octree.Update(new Vector3(0, 0, 0), new Vector3(5, 5, 5));
        AssertInt(octree.Points.Count).IsEqual(1);
        var point = octree.Points[0];
        AssertVector(point).IsEqual(new Vector3(5, 5, 5));
    }
}