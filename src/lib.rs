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
            self.integral +
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
    } else if -value < -value_max {
        return -value_max;
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
