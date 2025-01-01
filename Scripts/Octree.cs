using System.Collections.Generic;
using Godot;

namespace Boids.Scripts;

public class Octree
{
    public Aabb Bounds;
    public Octree[]? Children;
    public List<Vector3> Points;
    private int _capacity;
    private readonly object _lock = new();
    
    public Octree(Vector3 center, Vector3 size, int capacity)
    {
        Bounds = new Aabb(center - size * 0.5f, size);
        _capacity = capacity;
        Points = new List<Vector3>();
        Children = null;
    }

    public bool Insert(Vector3 point)
    {
        lock (_lock)
        {
            if (!Bounds.HasPoint(point))
            {
                //GD.Print($"Tried to insert point {point} into octree {Bounds}, FAIL");
                return false;
            }

            if (Children != null)
            {
                foreach (var child in Children)
                {
                    if (child.Insert(point)) // Recursive insert
                    {
                        //GD.Print($"Inserting {point} into octree {Bounds}, SUCCESS");
                        return true;
                    }
                }
                // Should never reach here
                GD.PrintErr("Point insertion failed for ", point);
                return false;
            }
        
            Points.Add(point);
            //GD.Print($"Inserting {point} into octree {Bounds}, SUCCESS");
            if (Points.Count >= _capacity)
            {
                Subdivide();
            }
            return true;
        }
    }

    public List<Vector3> Query(Vector3 center, Vector3 size)
    {
        lock (_lock)
        {
            var result = new List<Vector3>();
            var bounds = new Aabb(center - size * 0.5f, size);
            if (!Bounds.Intersects(bounds))
            {
                return result;
            }

            foreach (var point in Points)
            {
                if (Bounds.HasPoint(point))
                {
                    result.Add(point);
                }
            }

            if (Children != null)
            {
                foreach (var child in Children)
                {
                    result.AddRange(child.Query(center, size));
                }
            }

            return result;
        }
    }
    
    public void Update(Vector3 oldPosition, Vector3 newPosition)
    {
        lock(_lock) 
        {
            if (!Remove(oldPosition))
            {
                GD.PrintErr($"Failed to remove point {oldPosition} for update.");
                return;
            }

            if (!Insert(newPosition))
            {
                GD.PrintErr($"Failed to insert updated point {newPosition}.");
            }
        }
    }

    private bool Remove(Vector3 point)
    {
        if (!Bounds.HasPoint(point))
        {
            return false;
        }

        if (Children != null)
        {
            foreach (var child in Children)
            {
                if (child.Remove(point))
                {
                    CollapseIfEmpty();
                    return true;
                }
            }

            return false;
        }

        return Points.Remove(point);
    }
    
    private void CollapseIfEmpty()
    {
        if (Children == null)
            return;

        var allChildrenEmpty = true;
        foreach (var child in Children)
        {
            if (child.Points.Count > 0 || child.Children != null)
            {
                allChildrenEmpty = false;
                break;
            }
        }

        if (allChildrenEmpty)
        {
            Children = null;
            GD.Print($"Collapsed octree node at {Bounds} due to empty children.");
        }
    }

    private void Subdivide()
    {
        // Create 8 children
        Children = new Octree[8];
        var halfSize = Bounds.Size * 0.5f;
        for (var i = 0; i < 8; i++)
        {
            var offset = new Vector3(
                (i & 1) == 0 ? -halfSize.X * 0.5f : halfSize.X * 0.5f,
                (i & 2) == 0 ? -halfSize.Y * 0.5f : halfSize.Y * 0.5f,
                (i & 4) == 0 ? -halfSize.Z * 0.5f : halfSize.Z * 0.5f
            );
            Children[i] = new Octree(Bounds.GetCenter() + offset, halfSize, _capacity);
            //GD.Print($"Parent bounds are {Bounds}, child bounds are {Children[i].Bounds}");
        }
        // Reassign points to children
        foreach (var point in Points)
        {
            foreach (var child in Children)
            {
                if (child.Insert(point))
                    break;
            }
        }
        // Clear points, they're now in children
        Points.Clear();
    }

    public int CountNodes(Octree? node = null)
    {
        node ??= this;
        var count = 1; // Count this node
        if (node.Children != null)
        {
            foreach (var child in node.Children)
            {
                count += CountNodes(child);
            }
        }

        return count;
    }

    public int CountPoints(Octree? node = null)
    {
        node ??= this;
        var count = node.Points.Count;
        if (node.Children != null)
        {
            foreach (var child in node.Children)
            {
                count += CountPoints(child);
            }
        }
        
        return count;
    }

    public List<Aabb> GetAllBounds()
    {
        var bounds = new List<Aabb>();
        bounds.Add(Bounds);
        if (Children != null)
        {
            foreach (var child in Children)
            {
                bounds.AddRange(child.GetAllBounds());
            }
        }
        return bounds;
    }
}