//package org.firstinspires.ftc.teamcode.RoadRunner;
//
//public class XYPlace {
//
//    delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos
//            delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos
//    delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos
//
//            phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth
//    delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2
//    delta_perp_pos = delta_center_encoder_pos - forward_offset * phi
//
//            delta_x = delta_middle_pos * cos(heading) - delta_perp_pos * sin(heading)
//    delta_y = delta_middle_pos * sin(heading) + delta_perp_pos * cos(heading)
//
//    x_pos += delta_x
//    y_pos += delta_y
//    heading += phi
//
//            prev_left_encoder_pos = left_encoder_pos
//    prev_right_encoder_pos = right_encoder_pos
//            prev_center_encoder_pos = center_encoder_pos
//}
