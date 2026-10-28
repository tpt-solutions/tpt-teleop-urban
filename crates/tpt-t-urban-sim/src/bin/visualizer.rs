//! Visualizer for the crossing scenario. Built with the `viz` feature:
//! `cargo run -p tpt-t-urban-sim --features viz --bin visualizer`.
//!
//! Draws the frames from `tpt_t_urban_sim::viz`: the lane, the vehicle (red
//! while the guardian intervenes), the pedestrian, and a ring on contact. A
//! slider scrubs through the run, and Play advances it. The window is not
//! checked visually here. The frame model it draws has its own tests.

use eframe::egui;
use tpt_t_urban_sim::scenario::{LANE_HALF_WIDTH_M, Scenario};
use tpt_t_urban_sim::viz::{Bounds, Frame, bounds, frames};

/// Vehicle length and width used for the drawing, in metres.
const VEHICLE_LENGTH_M: f32 = 4.5;
const VEHICLE_WIDTH_M: f32 = 1.8;

struct Visualizer {
    frames: Vec<Frame>,
    bounds: Bounds,
    index: usize,
    playing: bool,
}

impl Visualizer {
    fn new() -> Self {
        let scenario = Scenario::default();
        let frames = frames(&scenario);
        let bounds = bounds(&frames);
        Self {
            frames,
            bounds,
            index: 0,
            playing: false,
        }
    }

    /// Maps a world point to the screen rectangle, with north up.
    fn to_screen(&self, rect: egui::Rect, x: f32, y: f32) -> egui::Pos2 {
        let [x0, x1] = [self.bounds.x[0] - 2.0, self.bounds.x[1] + 2.0];
        let [y0, y1] = [self.bounds.y[0] - 1.0, self.bounds.y[1] + 1.0];
        let u = (x - x0) / (x1 - x0);
        let v = (y - y0) / (y1 - y0);
        egui::pos2(
            rect.left() + u * rect.width(),
            rect.bottom() - v * rect.height(),
        )
    }

    fn draw(&self, ui: &egui::Ui, rect: egui::Rect, frame: &Frame) {
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_gray(245));

        // The lane band, from one edge to the other.
        let lane = egui::Rect::from_two_pos(
            self.to_screen(rect, self.bounds.x[0] - 2.0, LANE_HALF_WIDTH_M),
            self.to_screen(rect, self.bounds.x[1] + 2.0, -LANE_HALF_WIDTH_M),
        );
        painter.rect_filled(lane, 0.0, egui::Color32::from_gray(220));

        // The vehicle: the bumper is at `vehicle_x + FRONT_OFFSET`, the rear at
        // `vehicle_x + FRONT_OFFSET - length`.
        let front = frame.vehicle_x + tpt_t_urban_sim::scenario::FRONT_OFFSET_M;
        let vehicle = egui::Rect::from_two_pos(
            self.to_screen(rect, front - VEHICLE_LENGTH_M, VEHICLE_WIDTH_M / 2.0),
            self.to_screen(rect, front, -VEHICLE_WIDTH_M / 2.0),
        );
        let colour = if frame.intervening {
            egui::Color32::from_rgb(200, 40, 40)
        } else {
            egui::Color32::from_rgb(40, 90, 200)
        };
        painter.rect_filled(vehicle, 3.0, colour);

        // The pedestrian, with a ring while in contact.
        let [px, py] = frame.pedestrian;
        let at = self.to_screen(rect, px, py);
        painter.circle_filled(at, 6.0, egui::Color32::from_rgb(230, 140, 30));
        if frame.in_contact {
            painter.circle_stroke(at, 12.0, egui::Stroke::new(2.0, egui::Color32::RED));
        }
    }
}

impl eframe::App for Visualizer {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let last = self.frames.len().saturating_sub(1);
        if self.playing {
            self.index = (self.index + 1).min(last);
            if self.index == last {
                self.playing = false;
            }
            ui.ctx().request_repaint();
        }

        ui.horizontal(|ui| {
            let label = if self.playing { "Pause" } else { "Play" };
            if ui.button(label).clicked() {
                self.playing = !self.playing;
            }
            ui.add(egui::Slider::new(&mut self.index, 0..=last).text("step"));
        });

        let Some(frame) = self.frames.get(self.index).copied() else {
            ui.label("no frames");
            return;
        };
        ui.label(format!(
            "t {:.2} s   speed {:.2} m/s   gap {:.2} m   guardian {}   {}",
            frame.t_s,
            frame.speed_mps,
            frame.gap_m,
            if frame.intervening {
                "intervening"
            } else {
                "nominal"
            },
            if frame.in_contact { "CONTACT" } else { "" },
        ));

        let size = ui.available_size();
        let (rect, _response) = ui.allocate_exact_size(size, egui::Sense::hover());
        self.draw(ui, rect, &frame);
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "tpt-t-urban visualizer",
        options,
        Box::new(|_cc| Ok(Box::new(Visualizer::new()))),
    )
}
