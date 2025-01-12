using Godot;

namespace Boids.Scripts;

public partial class UI : Node
{
	[Export] private BoidSimulation _boidSimulation;
	[Export] private Slider _minSeparationDistance;
	[Export] private Slider _separationFactor;
	[Export] private Slider _alignmentFactor;
	[Export] private Slider _cohesionFactor;
	[Export] private Slider _visualRange;
	[Export] private Slider _maxSpeed;

	public override void _Ready()
	{
		_minSeparationDistance.Value = _boidSimulation.MinSeparationDistance;
		_separationFactor.Value = _boidSimulation.SeparationFactor;
		_alignmentFactor.Value = _boidSimulation.AlignmentFactor;
		_cohesionFactor.Value = _boidSimulation.CohesionFactor;
		_visualRange.Value = _boidSimulation.VisualRange;
		_maxSpeed.Value = _boidSimulation.MaxSpeed;
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
}