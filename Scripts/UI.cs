using Godot;

namespace Boids.Scripts;

public partial class UI : Node
{
	[Export] private BoidSimulation _boidSimulation;
	[Export] private SpinBox _minSeparationDistance;
	[Export] private SpinBox _separationFactor;
	[Export] private SpinBox _alignmentFactor;
	[Export] private SpinBox _cohesionFactor;
	[Export] private SpinBox _visualRange;
	[Export] private SpinBox _maxSpeed;
	[Export] private SpinBox _xBounds;
	[Export] private SpinBox _yBounds;
	[Export] private SpinBox _zBounds;

	public override void _Ready()
	{
		_minSeparationDistance.Value = _boidSimulation.MinSeparationDistance;
		_separationFactor.Value = _boidSimulation.SeparationFactor;
		_alignmentFactor.Value = _boidSimulation.AlignmentFactor;
		_cohesionFactor.Value = _boidSimulation.CohesionFactor;
		_visualRange.Value = _boidSimulation.VisualRange;
		_maxSpeed.Value = _boidSimulation.MaxSpeed;
		_xBounds.Value = _boidSimulation.Bounds.X;
		_yBounds.Value = _boidSimulation.Bounds.Y;
		_zBounds.Value = _boidSimulation.Bounds.Z;
	}
	
	private void OnMinSeparationDistanceChanged(float value)
	{
		_boidSimulation.MinSeparationDistance = value;
	}
	
	private void OnSeparationFactorChanged(float value)
	{
		_boidSimulation.SeparationFactor = value;
	}
	
	private void OnAlignmentFactorChanged(float value)
	{
		_boidSimulation.AlignmentFactor = value;
	}
	
	private void OnCohesionFactorChanged(float value)
	{
		_boidSimulation.CohesionFactor = value;
	}
	
	private void OnVisualRangeChanged(float value)
	{
		_boidSimulation.VisualRange = value;
	}
	
	private void OnMaxSpeedChanged(float value)
	{
		_boidSimulation.MaxSpeed = value;
	}
	
	private void OnXBoundsChanged(float value)
	{
		_boidSimulation.Bounds.X = value;
	}
	
	private void OnYBoundsChanged(float value)
	{
		_boidSimulation.Bounds.Y = value;
	}
	
	private void OnZBoundsChanged(float value)
	{
		_boidSimulation.Bounds.Z = value;
	}
}