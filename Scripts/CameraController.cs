using Godot;

namespace Boids.Scripts;

public partial class CameraController : Node3D
{
    [Export] private float _moveSpeed = 5f;
    [Export] private float _mouseSensitivity = 0.15f;
    [Export] private Camera3D _camera;
    [Export] private bool _cameraRotation = false;
    private Vector2 _mouseDelta;
    
    private float _pitch = 0f;
    private float _yaw   = 0f;

    public override void _Ready()
    {
        if(_cameraRotation)
            Input.MouseMode = Input.MouseModeEnum.Captured;
    }

    public override void _Input(InputEvent @event)
    {
        // Capture raw mouse movement
        if (_cameraRotation && @event is InputEventMouseMotion mouseMotion)
        {
            if (@event is InputEventMouseMotion motion)
            {
                // Yaw (left/right) rotate around Y axis
                _yaw -= motion.Relative.X * _mouseSensitivity;
            
                // Pitch (up/down) rotate around X axis
                _pitch -= motion.Relative.Y * _mouseSensitivity;
                _pitch = Mathf.Clamp(_pitch, -89f, 89f);

                _camera.RotationDegrees = new Vector3(_pitch, _yaw, 0f);
            }
            _mouseDelta = mouseMotion.Relative;
        }
    }

    public override void _Process(double delta)
    {
        var inputDirection = Input.GetVector("MoveLeft", "MoveRight", "MoveForward", "MoveBackward");
        var upDown = Input.GetAxis("MoveDown", "MoveUp");
        var direction = _camera.Basis * new Vector3(inputDirection.X, upDown, inputDirection.Y).Normalized();
        Translate(direction * _moveSpeed * (float)delta);
    }
}