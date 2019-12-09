use assert_approx_eq::assert_approx_eq;

pub struct Pid {
    terms: PidTerms,
    reverse_output: bool,
    update_freq: f32,
    integral: f32,
    integral_max: f32,
}

struct PidTerms {
    kp: f32,
    ki: f32,
    kd: f32
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, update_freq: f32, integral_max: f32) -> Pid {
        Pid {
            terms: PidTerms::new(kp, ki, kd, update_freq),
            reverse_output: false, // default
            update_freq,
            integral: 0.0,
            integral_max
        }
    }

    pub fn set_reverse_output(&mut self, reverse_output: bool) {
        self.reverse_output = reverse_output;
    }

    pub fn update_terms(&mut self, kp: f32, ki: f32, kd: f32) {
        self.terms = PidTerms::new(kp, ki, kd, self.update_freq);
    }

    pub fn update_integral_max(&mut self, integral_max: f32) {
        self.integral_max = integral_max;
        self.integral = truncate_pos_or_neg(self.integral, self.integral_max);
    }

    pub fn clear_integral(&mut self) {
        self.integral = 0.0;
    }

    pub fn get_output(&mut self, setpoint: f32, current_val: f32, last_val: f32) -> f32 {
        let error = setpoint - current_val;

        self.integral = self.integral + (self.terms.ki * error);
        self.integral = truncate_pos_or_neg(self.integral, self.integral_max);

        let output = self.terms.kp * error +
            self.integral -
            self.terms.kd * (current_val - last_val);

        if self.reverse_output {
            -output
        } else {
            output
        }
    }
}

impl PidTerms {
    fn new(kp: f32, ki: f32, kd: f32, update_freq: f32) -> PidTerms {
        let sample_time = 1.0f32 / update_freq;
        let ki = ki * sample_time;
        let kd = kd * update_freq;
        
        PidTerms { kp, ki, kd }
    }
}

fn truncate_pos_or_neg(value: f32, value_max: f32) -> f32 {
    if value > value_max {
        return value_max;
    } else if value < -value_max {
        return -value_max;
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_clear_integral() {
        let mut pid = Pid::new(100.0, 10.0, 1.0, 1000.0, 100.0);
        pid.integral = 50.0;
        pid.clear_integral();
        assert_eq!(0.0, pid.integral);
    }
    #[test]
    fn test_set_tunings() {
        let mut pid = Pid::new(100.0, 10.0, 1.0, 1000.0, 100.0);
        assert_approx_eq!(100.0, pid.terms.kp);
        assert_approx_eq!(0.01, pid.terms.ki);
        assert_approx_eq!(1000.0, pid.terms.kd);

        pid.update_terms(50.0, 5.0, 0.5);
        assert_approx_eq!(50.0, pid.terms.kp);
        assert_approx_eq!(0.005, pid.terms.ki);
        assert_approx_eq!(500.0, pid.terms.kd);
    }

    #[test]
    fn test_set_integral_max() {
        let mut pid = Pid::new(100.0, 10.0, 1.0, 1000.0, 100.0);
        assert_approx_eq!(100.0, pid.integral_max);
        pid.update_integral_max(60.0);
        assert_approx_eq!(60.0, pid.integral_max);

        // Test trimming
        pid.integral = 100.0;
        pid.update_integral_max(50.0);
        assert_approx_eq!(50.0, pid.integral);
        pid.integral = -100.0;
        pid.update_integral_max(50.0);
        assert_approx_eq!(-50.0, pid.integral);
    }

    #[test]
    fn test_get_output() {
        let mut pid = Pid::new(10.0, 5.0, 1.0, 500.0, 50.0);
        let output = pid.get_output(120.0, 110.0, 100.0);
        // P term -> 10 * 10 -> 100
        // I term -> 0.01 * 10 -> 0.1
        // D term -> 500 * -10 -> -5000
        assert_approx_eq!(-4899.9, output);
        let output = pid.get_output(120.0, 119.0, 118.0);
        // P term -> 10 * 1 -> 10
        // I term -> 0.1 + (0.01 * 1) -> 0.11
        // D term -> 500 * -1 -> -500
        assert_approx_eq!(-489.89, output);
    }

    #[test]
    fn test_set_polarity() {
        let mut pid = Pid::new(10.0, 5.0, 1.0, 500.0, 50.0);
        pid.set_reverse_output(true);

        let output = pid.get_output(120.0, 110.0, 100.0);
        assert_approx_eq!(4899.9, output);
    }
}
