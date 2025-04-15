namespace lab1
{
    public class ControlSystem
    {
        private PIDBlock _pid;
        private double _dt;
        private bool _isAutoMode = true;
        private double _manualOutput = 0;
        private double _processVariable = 0;

        public double SetPoint { get; set; }
        public double Kp { get => _pid.Kp; set => _pid.Kp = value; }
        public double Ki { get => _pid.Ki; set => _pid.Ki = value; }
        public double Kd { get => _pid.Kd; set => _pid.Kd = value; }
        public bool IsAutoMode => _isAutoMode;
        public double Output { get; private set; }
        public double ProcessVariable => _processVariable;

        public ControlSystem(double dt)
        {
            _dt = dt;
            _pid = new PIDBlock(1.0, 0.1, 0.01, dt);
        }

        public void SwitchToAutoMode()
        {
            if (!_isAutoMode)
            {
                double error = SetPoint - _processVariable;
                _pid.SetManualOutput(_manualOutput, error);
                _isAutoMode = true;
            }
        }

        public void SwitchToManualMode(double manualOutput)
        {
            if (_isAutoMode)
            {
                _isAutoMode = false;
            }
            _manualOutput = manualOutput;
        }

        public double ComputeControlOutput(double processVariable)
        {
            _processVariable = processVariable;

            if (_isAutoMode)
            {
                double error = SetPoint - processVariable;
                Output = _pid.Transfer(error);
            }
            else
            {
                Output = _manualOutput;
            }

            return Output;
        }
    }
}