/**
 * \file viz.hpp
 * \brief Class for visualizing inputs and outputs.
 *
 * This file defines the Viz class which is used for displaying
 * maps, generated paths and points on the map
 */
#ifndef MPEL_VIZ_H
#define MPEL_VIZ_H

#include <opencv2/opencv.hpp>

#define COLOR_BLUE cv::Scalar(255, 0, 0)
#define COLOR_GREEN cv::Scalar(0, 255, 0)
#define COLOR_RED cv::Scalar(0, 0, 255)
#define COLOR_DARK_GREEN cv::Scalar(0, 100, 0)
#define COLOR_PINK cv::Scalar(211, 0, 246)
#define COLOR_GRAY cv::Scalar(150, 150, 150)

#include "planner.hpp"
#include "types.hpp"
#include <string>

namespace mpel {
class View {
public:
    /// Constructor for Viz class
    View(std::string name = "View: " + std::to_string(nviews));

    /// Close the visualizer
    void close();

    /// Clear all the entities displayed on visualizer
    void clear();

    /// Add a map to the display
    void add_layer(MapRef map);
    /// Add a point on the display
    void add_layer(PointRef pt);
    /// Display the start and end points on display
    void add_layer(ProblemDefinition pdef);
    /// Add a path to the display
    void add_layer(PathRef path);
    /// Display a graph
    void add_layer(GraphRef graph);
    /// Convenience function, displays the map, graph
    void add_layer(const Planner& p);

    /// Save the current display to a file
    void save(std::string filename = "view.png");

    /// Keep the program from exiting
    static void stay(char key = ' ');

private:
    cv::Mat _img;

    std::string _name;

    void update();

    static size_t nviews;
};
}
#endif
