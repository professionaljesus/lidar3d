void spherical_ground_removal(vector<vector<float>> &r_matrix, vector<vector<float>> &theta_matrix, vector<vector<int>> &r_matrix_nonzero_indices, vector<vector<Vector3f>> &cart_matrix, float h)
{
    ground_segment_cloud.clear();
    object_segment_cloud.clear();
    azimuth_group_cloud.clear();
    cluster_cloud.clear();
    cluster_with_reattached_ground_cloud.clear();

    vector<list<int>> break_points(16);
    vector<vector<Line_segment>> line_segments(16);
    vector<vector<Line_segment>> combined_line_segments(16);

    for (int i = 0; i < 16; i++)
    {
        float r1, r2;
        float t1, t2;

        //add first break point
        break_points[i].push_back(0);

        for (int j = 1; j < r_matrix_nonzero_indices[i].size(); j++)
        {
            r1 = r_matrix[i][r_matrix_nonzero_indices[i][j - 1]];
            r2 = r_matrix[i][r_matrix_nonzero_indices[i][j]];

            t1 = theta_matrix[i][r_matrix_nonzero_indices[i][j - 1]];
            t2 = theta_matrix[i][r_matrix_nonzero_indices[i][j]];

            D_MAX_BREAKPOINTS = min(r1, r2) * (sin(LAMBDA) / sin(LAMBDA - abs(t1 - t2)) - 1) + EPSILON;

            if (abs(r1 - r2) > D_MAX_BREAKPOINTS)
            {
                break_points[i].push_back(j - 1); //end of segment
                break_points[i].push_back(j);     //begining of new segment
            }
        }

        //add last break point
        break_points[i].push_back(r_matrix_nonzero_indices[i].size() - 1);

        int n_s_index, n_e_index, n_s, n_e;
        while (break_points[i].size() > 0)
        {
            n_s_index = break_points[i].front();
            break_points[i].pop_front();
            n_e_index = break_points[i].front();
            break_points[i].pop_front();

            n_s = r_matrix_nonzero_indices[i][n_s_index];
            n_e = r_matrix_nonzero_indices[i][n_e_index];

            //Is the segment large enoguh?
            if (n_e - n_s + 1 > MIN_SEGMENT_LENGTH)
            {
                Eigen::Vector3f abc = alpha_beta(theta_matrix[i][n_s], theta_matrix[i][n_e], r_matrix[i][n_s], r_matrix[i][n_e], h, phis[i]);
                float dj, wj;
                float dmax = -__FLT_MAX__;
                int dmax_index = -1;
                int k;

                //Find outliers that dont fit the line
                for (int j = n_s_index + 1; j < n_e_index; j++)
                {
                    k = r_matrix_nonzero_indices[i][j];

                    wj = theta_matrix[i][k];
                    dj = abs(h - expected_height(abc, wj, phis[i], r_matrix[i][k]));
                    if (dj > dmax)
                    {
                        dmax = dj;
                        dmax_index = j;
                    }
                }

                //If largest outlier is bigger than the threshold, split into two segments

                if (dmax > D_THRESHHOLD)
                {
                    //second segment
                    break_points[i].push_front(n_e_index);
                    break_points[i].push_front(dmax_index);

                    //first segment
                    break_points[i].push_front(dmax_index - 1);
                    break_points[i].push_front(n_s_index);
                }
                else
                { //Add as valid line segment
                    bool object = abs(abc.x() - ALPHA) > ALPHA_RANGE || abs(abc.y() - BETA) > BETA_RANGE && n_e - n_s + 1 < points_at_distance(r_matrix[i][(n_e + n_s) / 2], 0.20, 0.2 * (M_PI) / 180);
                    Line_segment temp_l{n_s_index, n_e_index, abc, object};
                    line_segments[i].push_back(temp_l);
                }
            }
            //Combine line segments
               if(combined_line_segments[i].size == 0 ) continue;
               combined_line_segments[i].push_back(line_segments[i][0]);
            //Combination of adjacent line segments v1
            for (int j = 1; j < line_segments[i].size(); j++){
            Line_segment l1 = line_segments[i][j-1];
            Line_segment l2 = line_segments[i][j];

            if(l2.start - l1.end < INDEX_THRESHOLD && abs(l1.normal.x() - l2.normal.x()) < ALPHA_COMBINATION_THRESHOLD 
            && abs(l1.normal.y() - l2.normal.y()) <  BETA_COMBINATION_THRESHOLD){

            n_s = r_matrix_nonzero_indices[i][l1.start];
            n_e = r_matrix_nonzero_indices[i][l2.end];
            Eigen::Vector3f abc = alpha_beta(theta_matrix[i][n_s], theta_matrix[i][n_e], cloud[i][n_s], cloud[i][n_e], phis[i]);
            combined_line_segments[i].emplace_back(l1.start,l2.end,abc);
            }else{
            combined_line_segments[i].push_back(l1);
            combined_line_segments[i].push_back(l2);
            }
            }
        }

        for (int j = 0; j < line_segments[i].size(); j++)
        {
            Line_segment l = line_segments[i][j];

            pcl::PointXYZRGB p;
            float phi = phis[i];

            int a = r_matrix_nonzero_indices[i][l.start];
            float theta = theta_matrix[i][a];
            float r = r_matrix[i][a];
            Vector3f polar(r, theta, phi);
            Vector3f cart = cart_matrix[i][a];
            p.x = cart.x();
            p.y = cart.y();
            p.z = cart.z();
            p.r = 0;
            p.g = 255;
            p.b = 0;
            if (l.object)
            {
                object_segment_cloud.points.push_back(p);
            }
            else
            {
                ground_segment_cloud.points.push_back(p);
            }

            //string text = "x: " + to_string(l.normal.x()) + " y: " + to_string(l.normal.y());
            //markers.markers.push_back(text_marker(cart.x(),cart.y(), cart.z() + 0.05,text, 0.05, "alpha_beta"));

            a = r_matrix_nonzero_indices[i][l.end];
            theta = theta_matrix[i][a];
            r = r_matrix[i][a];
            polar << r, theta, phi;
            cart = cart_matrix[i][a];
            p.x = cart.x();
            p.y = cart.y();
            p.z = cart.z();
            p.r = 255;
            p.g = 0;
            p.b = 0;
            if (l.object)
            {
                object_segment_cloud.points.push_back(p);
            }
            else
            {
                ground_segment_cloud.points.push_back(p);
            }

            for (int b = l.start + 1; b < l.end - 1; b++)
            {
                a = r_matrix_nonzero_indices[i][b];
                theta = theta_matrix[i][a];
                r = r_matrix[i][a];
                polar << r, theta, phi;
                cart = cart_matrix[i][a];
                p.x = cart.x();
                p.y = cart.y();
                p.z = cart.z();
                p.r = 0;
                p.g = 0;
                p.b = 255;
                if (l.object)
                {
                    object_segment_cloud.points.push_back(p);
                }
                else
                {
                    ground_segment_cloud.points.push_back(p);
                }

                //Estimated point
                r = compute_radius(l.normal, theta, h, phi);
                polar << r, theta, phi;
                cart = cart_matrix[i][a];
                p.x = cart.x();
                p.y = cart.y();
                p.z = cart.z();
                p.r = 255;
                p.g = 255;
                p.b = 255;
                if (l.object)
                {
                    object_segment_cloud.points.push_back(p);
                }
                else
                {
                    ground_segment_cloud.points.push_back(p);
                }
            }
        }
    }

    //Mark the points of the object segments
    vector<vector<bool>> object_points(16, vector<bool>(1824, false));
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < line_segments[i].size(); j++)
        {
            Line_segment l = line_segments[i][j];
            if (!l.object)
                continue;
            for (int k = l.start; k < l.end; k++)
            {
                int m = r_matrix_nonzero_indices[i][k];
                object_points[i][m] = true;
            }
        }
    }

    //Azimuth grouping
    vector<vector<pair<int, int>>> azimuth_groups;
    int last_azimuth_index = -AZIMUTH_INDEX_THRESHOLD;
    pcl::PointXYZRGB p;

    //Loop over the points (bot->top for left->right)
    for (int i = 0; i < 1824; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            if (!object_points[j][i])
                continue;

            //Start new group
            if (i - last_azimuth_index >= AZIMUTH_INDEX_THRESHOLD)
            {
                vector<pair<int, int>> v;
                azimuth_groups.push_back(v);
            }
            last_azimuth_index = i;
            azimuth_groups.back().emplace_back(j, i);

            //Visualizing
            Vector3f cart = cart_matrix[j][i];
            p.x = cart.x();
            p.y = cart.y();
            p.z = cart.z();
            p.r = 0;
            p.g = 0;
            p.b = 0;
            if (azimuth_groups.size() % 3 == 0)
                p.r = 255;
            if (azimuth_groups.size() % 3 == 1)
                p.g = 255;
            if (azimuth_groups.size() % 3 == 2)
                p.b = 255;
            azimuth_group_cloud.points.push_back(p);
        }
    }

    //Radial clustering
    vector<vector<Cluster>> radial_clusters(azimuth_groups.size());
    int a, b, c, i, j, closest_cluster_index;
    float r_diff, min_r_diff, alpha;
    Cluster *closest_cluster;

    //Loop over the azimuth groups
    for (a = 0; a < azimuth_groups.size(); a++)
    {

        //Loop over the points in the group
        for (b = 0; b < azimuth_groups[a].size(); b++)
        {
            i = azimuth_groups[a][b].first;
            j = azimuth_groups[a][b].second;
            min_r_diff = __FLT_MAX__;

            //Loop over the radial_clusters and find the closest
            for (c = 0; c < radial_clusters[a].size(); c++)
            {
                r_diff = abs(radial_clusters[a][c].radial_center - r_matrix[i][j]);
                if (r_diff < min_r_diff)
                {
                    min_r_diff = r_diff;
                    closest_cluster = &radial_clusters[a][c];
                }
            }

            if (min_r_diff < RADIAL_THRESHOLD)
            { //Add point to the closest cluster and update center
                closest_cluster->points.push_back(azimuth_groups[a][b]);
                alpha = 1.0 / closest_cluster->points.size();
                closest_cluster->radial_center = (1 - alpha) * closest_cluster->radial_center + alpha * r_matrix[i][j];
                closest_cluster->center = (1 - alpha) * closest_cluster->center + alpha * cart_matrix[i][j];

                int added_index = closest_cluster->points.size() - 1;

                pair<int, int> &top_point = (closest_cluster->points[closest_cluster->top]);
                pair<int, int> &bottom_point = (closest_cluster->points[closest_cluster->bottom]);
                if (cart_matrix[i][j].z() > cart_matrix[top_point.first][top_point.second].z())
                {
                    closest_cluster->top = added_index;
                }

                if (cart_matrix[i][j].z() < cart_matrix[bottom_point.first][bottom_point.second].z())
                {
                    closest_cluster->bottom = added_index;
                }

                if (j > closest_cluster->points[closest_cluster->right].second)
                {
                    closest_cluster->right = added_index;
                }
            }
            else
            { //Create new cluster
                Cluster cluster;
                cluster.radial_center = r_matrix[i][j];
                cluster.center = cart_matrix[i][j];
                cluster.right = 0;
                cluster.left = 0;
                cluster.top = 0;
                cluster.bottom = 0;
                cluster.points.push_back(azimuth_groups[a][b]);
                radial_clusters[a].push_back(cluster);
            }
        }
    }

    //Visualize the radial_clusters
    Vector3f cart;
    int cluster_counter = 0;
    for (a = 0; a < radial_clusters.size(); a++)
    {
        for (b = 0; b < radial_clusters[a].size(); b++)
        {
            for (c = 0; c < radial_clusters[a][b].points.size(); c++)
            {
                i = radial_clusters[a][b].points[c].first;
                j = radial_clusters[a][b].points[c].second;
                cart = cart_matrix[i][j];
                p.x = cart.x();
                p.y = cart.y();
                p.z = cart.z();
                p.r = 0;
                p.g = 0;
                p.b = 0;
                if (cluster_counter % 3 == 0)
                    p.r = 255;
                if (cluster_counter % 3 == 1)
                    p.g = 255;
                if (cluster_counter % 3 == 2)
                    p.b = 255;
                cluster_cloud.points.push_back(p);
            }
            cluster_counter++;
        }
    }

    //Reattach ground
    for (a = 0; a < radial_clusters.size(); a++)
    {
        for (vector<Cluster>::iterator it = radial_clusters[a].begin(); it != radial_clusters[a].end(); it++)
        {
            reattach_ground(&(*it), r_matrix, cart_matrix);
        }
    }
    VectorXf plane_coeff(4);

    plane_coeff << 0, 0, 1, 0;

    for (a = 0; a < radial_clusters.size(); a++)
    {
        for (vector<Cluster>::iterator it = radial_clusters[a].begin(); it != radial_clusters[a].end();)
        {
            if (classify_cluster(&(*it), cart_matrix, plane_coeff) < 0.5)
            {
                radial_clusters[a].erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
}

VectorXf best_plane_from_points(const list<Vector3f> &c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    
    Eigen::MatrixXf coord(3, num_atoms);
    size_t i = 0;
    for (Vector3f cc : c) 
    {
        coord.col(i) = cc;
        i++;
    }

    // calculate centroid
    Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3f plane_normal = svd.matrixU().rightCols<1>();
    float d = -(plane_normal[0]*centroid[0] + plane_normal[1]*centroid[1] + plane_normal[1]*centroid[1]);
    VectorXf plane_coeff(4); 
    plane_coeff << plane_normal.x(), plane_normal.y(), plane_normal.z(), d;

    return plane_coeff;
}



VectorXf ground_removal(vector<vector<float>> &r_matrix, vector<vector<Vector3f>> &cart_matrix, vector<pair<int, int>> &after_ransac_indices, vector<list<int>> &r_matrix_nonzero_indices)
{
    VectorXf plane_coeff;
    //Ransac
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>::Ptr
        model_p(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZI>(possible_ground_points));
    Eigen::Vector3f dir(0, 0, 1);
    model_p->setAxis(dir);
    model_p->setEpsAngle(PLANE_RANSAC_ANGLE_TRESHHOLD);
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p, PLANE_RANSAC_TRESHHOLD);
    ransac.setMaxIterations(PLANE_RANSAC_MAX_ITERATIONS);
    ransac.setProbability(PLANE_RANSAC_PROBABILITY);
    ransac.computeModel();

    ransac.getModelCoefficients(plane_coeff);
    if (plane_coeff[2] < 0)
    {
        for (int i = 0; i < 4; i++)
            plane_coeff[i] = -plane_coeff[i];
    }

    //Visualize the Ransac plane
    markers.markers.push_back(visualize_ransac(plane_coeff[0], plane_coeff[1], plane_coeff[2], plane_coeff[3], PLANE_RANSAC_EXTRACT_TRESHHOLD));

    //Extract points above plane
    after_ransac_indices.clear();
    after_ransac_cloud.clear();
    float temp = sqrt(plane_coeff[0] * plane_coeff[0] + plane_coeff[1] * plane_coeff[1] + plane_coeff[2] * plane_coeff[2]);
    pcl::PointXYZI p;

    for (size_t i = 0; i < r_matrix_nonzero_indices.size(); i++)
    {
        for (int j : r_matrix_nonzero_indices[i])
        {

            p.x = cart_matrix[i][j].x();
            p.y = cart_matrix[i][j].y();
            p.z = cart_matrix[i][j].z();

            if (plane_coeff[0] * p.x + plane_coeff[1] * p.y + plane_coeff[2] * p.z + plane_coeff[3] < PLANE_RANSAC_EXTRACT_TRESHHOLD * temp)
                continue;

            after_ransac_cloud.push_back(p);
            after_ransac_indices.emplace_back(i, j);
        }
    }

    return plane_coeff;
}



vector<pair<int, int>> amz_ground_removal(vector<vector<float>> &r_matrix, vector<vector<float>> &theta_matrix, vector<vector<Vector3f>> &cart_matrix)
{
    size_t n_lasers = r_matrix.size();
    size_t n_azimuth = r_matrix[0].size();

    vector<pair<int, int>> remaining_indices;
    size_t bin_size = n_azimuth/16;

    vector<list<Vector3f>> min_points(n_lasers);

    auto cmp = [](Vector3f l, Vector3f r){ return l.z()<r.z(); };

    for(size_t jj = 500; jj < n_azimuth - 500; jj += bin_size)
    {

        VectorXf most_likely_plane;
        VectorXf plane_coeff(4);
        plane_coeff << 0,0,1,0.19;
        for(size_t i = 0; i < n_lasers; i++)
        {

            
            min_points[i].clear();
            for(size_t j = jj; j < jj + bin_size; j++)
            {
                if(r_matrix[i][j] > 0)
                    min_points[i].push_back(cart_matrix[i][j]);
            }

            min_points[i].sort(cmp);
            
            list<Vector3f> reg_points;

            if(i >= 2 && min_points[i].size() > 0) 
            {
                vector<list<Vector3f>::iterator> its(3);

                for(size_t it_idx = 0; it_idx < its.size(); it_idx++)
                {
                    its[it_idx] = min_points[i - it_idx].begin();
                    for(size_t j = 0; j < bin_size/10 && its[it_idx] != min_points[i - it_idx].end(); j++)
                    {
                        reg_points.push_back(*its[it_idx]);
                        its[it_idx]++;
                    }
                }

                VectorXf new_plane_coeff = best_plane_from_points(reg_points);

                if (new_plane_coeff[2] < 0)
                {
                    new_plane_coeff *= -1;
                }

                float diff = (plane_coeff.head(3).dot(new_plane_coeff.head(3)));

                plane_coeff = new_plane_coeff;

            
                Vector3f point_in_ground = *next(min_points[i - 1].begin(), min_points[i - 1].size()/2);
                Vector3f rgb(1,0,0);
                visualization_msgs::Marker normal = arrow_marker(point_in_ground + plane_coeff.head(3), point_in_ground, rgb, "normal");
                markers.markers.push_back(normal);

                float temp = sqrt(plane_coeff[0] * plane_coeff[0] + plane_coeff[1] * plane_coeff[1] + plane_coeff[2] * plane_coeff[2]);

                pcl::PointXYZI p;
                for(size_t j = jj; j < jj + bin_size; j++) 
                {
                    if (r_matrix[i - 1][j] == 0)
                        continue;

                    p.x = cart_matrix[i - 1][j].x();
                    p.y = cart_matrix[i - 1][j].y();
                    p.z = cart_matrix[i - 1][j].z();

                    if (plane_coeff[0] * p.x + plane_coeff[1] * p.y + plane_coeff[2] * p.z + plane_coeff[3] < PLANE_RANSAC_EXTRACT_TRESHHOLD * temp)
                        continue;

                    after_ransac_cloud.push_back(p);
                    remaining_indices.emplace_back(i - 1, j);
                }

            }


        }
    }

    return remaining_indices;
}