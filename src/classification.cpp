#include "cluster.h"

using namespace std;
using namespace Eigen;

extern visualization_msgs::MarkerArray markers;
void color_classification(vector<Cluster>& clusters, vector<vector<Vector3f>>& cart_matrix, vector<vector<float>>& intensity_matrix)
{
    for (Cluster& cluster : clusters) {

        if (cluster.big_cone) {
            continue;
        }
        size_t cluster_size = calc_cluster_size(&cluster);
        //Matri    Vector2f alpha_beta = (A.transpose() * A).ldlt().solve(A.transpose() * bb);
        int lines = 0;
        list<float> avg_line_intensity;
        list<float> layers;

        for (size_t i = 0; i < cluster.points.size(); i++) {
            if (cluster.points[i].size() > 0) {
                avg_line_intensity.push_back(0);
                layers.push_back(((float)i) / 16.0);
                lines++;
                if (cluster.points[i].size() > 2) {
                    int counter = 0;
                    for (int j : cluster.points[i]) {
                        counter++;
                        if (counter > 0 && counter < cluster.points[i].size() - 1) {
                            avg_line_intensity.back() += intensity_matrix[i][j];
                        }
                    }
                    avg_line_intensity.back() = avg_line_intensity.back() / ((float)(cluster.points[i].size() - 2));
                } else {
                    for (int j : cluster.points[i]) {
                        avg_line_intensity.back() += intensity_matrix[i][j] / ((float)cluster.points[i].size());
                    }
                }
            }
        }

        if (lines < 3)
            continue;

        MatrixXf X(layers.size(), 3);
        VectorXf I(layers.size());

        string debug_text = "";
        int i = 0;
        for (float layer : layers) {
            X(i, 0) = pow(layer, 2);
            X(i, 1) = layer;
            X(i, 2) = 1.0;
            i++;
        }
        i = 0;
        for (float intensity : avg_line_intensity) {
            I(i) = intensity / 1024.0;
            i++;
            debug_text = to_string(intensity) + "\n" + debug_text;
        }
        Vector3f abc = (X.transpose() * X).ldlt().solve(X.transpose() * I);

        if (abc[0] > 0) {
            cluster.color = 1;
        } else if (abc[0] < 0) {
            cluster.color = 0;
        }

        markers.markers.push_back(text_marker(cluster.center.x(), cluster.center.y(), cluster.center.z() + 0.19, to_string(abc[0]), 0.1, "color_from_lidar"));
        markers.markers.push_back(text_marker(cluster.center.x(), cluster.center.y() + 0.2, cluster.center.z() + 0.39, debug_text, 0.1, "color_from_lidar2"));
    }
}
