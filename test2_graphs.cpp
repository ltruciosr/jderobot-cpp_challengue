#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct Point
{
    int x;  // pose in x
    int y;  // pose in y
    Point(int x_, int y_)
    {
        x = x_;
        y = y_;
    }
};

class Graph
{
   private:
    std::vector<Point> points_;
    std::vector<std::vector<int>> edges_;
    std::vector<int> start_;

    std::vector<std::vector<int>> paths_;
    int w_ = 0;
    int h_ = 0;
    std::vector<std::vector<int>> map_;

   public:
    void printMap()
    {
        // Get best path
        int max_size = 0;
        int best;
        for (int idx = 0; idx < paths_.size(); idx++)
        {
            int size = paths_[idx].size();
            if (size > max_size)
            {
                max_size = size;
                best = idx;
            }
        }

        // Format as input
        std::vector<int> best_path = paths_[best];
        std::vector<int> empty_path;
        for (int node = 0; node < points_.size(); node++)
        {
            if (std::find(best_path.begin(), best_path.end(), node) !=
                best_path.end())
                continue;

            empty_path.push_back(node);
        }

        for (int i = 0; i < best_path.size(); i++)
        {
            Point p = points_[best_path[i]];
            map_[p.x][p.y] = i;
        }

        for (int node : empty_path) map_[points_[node].x][points_[node].y] = -1;

        for (int row = 0; row < h_; row++)
        {
            for (int col = 0; col < w_; col++)
            {
                if (map_[col][row] >= 0)
                {
                    std::cout << "|" << map_[col][row] << "|";
                    continue;
                }
                else if (map_[col][row] == -1)
                {
                    std::cout << "|.|";
                    continue;
                }
                else
                    std::cout << "|#|";
            }
            std::cout << std::endl;
        }

        // Clean Map
        cleanMap();
    }

    void printPaths()
    {
        std::cout << "PATHS: " << std::endl;
        for (std::vector<int> path : paths_)
        {
            std::cout << "[";
            for (int node : path) std::cout << node << ", ";
            std::cout << "]" << std::endl;
        }
    }

    void printStart()
    {
        for (int idx : start_)
        {
            std::cout << "Start [" << idx << "]: (" << points_[idx].x << ", "
                      << points_[idx].y << ")" << std::endl;
        }
    }

    void printNodes()
    {
        for (size_t idx = 0; idx < points_.size(); idx++)
        {
            std::cout << "Node [" << idx << "]: (" << points_[idx].x << ", "
                      << points_[idx].y << ")" << std::endl;
        }
    }

    void printEdges()
    {
        for (size_t idx = 0; idx < edges_.size(); idx++)
        {
            std::cout << "Edges to Node [" << idx << "]: ";
            for (int adjacent : edges_[idx]) std::cout << adjacent << " ";
            std::cout << std::endl;
        }
    }

    void cleanMap()
    {
        for (int x = 0; x < w_; x++)
        {
            for (int y = 0; y < h_; y++)
            {
                map_[x][y] = -2;
            }
        }
    }

    void setSize(int w, int h)
    {
        w_ = w;
        h_ = h;

        map_.resize(w);
        for (int x = 0; x < w; x++) map_[x].resize(h);

        cleanMap();
    }

    bool isStart(Point& p1)
    {
        if (p1.y == 0)
            return true;
        else
            return false;
    }

    bool isAdjacent(Point& p1, Point& p2)
    {
        int delta = abs(p1.x - p2.x) + abs(p1.y - p2.y);
        if (delta == 1)
            return true;
        else
            return false;
    }

    void computeEdges()
    {
        // Resize Adj vector
        edges_.resize(points_.size());
        for (int i = 0; i < points_.size(); i++)
        {
            Point node_i = points_[i];
            for (int j = i + 1; j < points_.size(); j++)
            {
                Point node_j = points_[j];
                if (isAdjacent(node_i, node_j))
                {
                    edges_[i].push_back(j);
                    edges_[j].push_back(i);
                }
            }
            if (isStart(node_i)) start_.push_back(i);
        }
    }

    void addNode(int x, int y)
    {
        Point node(x, y);
        points_.push_back(node);
    }

    void computePath(int parent, std::vector<bool> visited,
                     std::vector<int> path)
    {
        // Init
        visited[parent] = true;
        path.push_back(parent);

        // Process
        std::vector<int> not_visited;
        for (int child : edges_[parent])
        {
            if (!visited[child]) not_visited.push_back(child);
        }

        // Propagate
        int num_childs = not_visited.size();
        if (num_childs == 0)
        {
            paths_.push_back(path);
        }
        else if (num_childs > 0)
        {
            for (int child : not_visited) computePath(child, visited, path);
        }

        // Clear
        visited.clear();
        path.clear();
    }

    void Propagate()
    {
        int n = points_.size();
        std::vector<bool> visited(n, false);  // according to connections
        std::vector<int> path;  // path generated until there is no more options
        for (int parent : start_)
        {
            computePath(parent, visited, path);
        }
    }
};

int main()
{
    std::ifstream file("filename");
    std::string line;

    // Create Map
    int h = 0;
    int w = 0;

    Graph Map;
    while (std::getline(file, line))
    {
        // Traverse the string
        for (int x = 0; x < line.size(); x++)
        {
            if (line[x] == '.') Map.addNode(x, h);
        }
        h++;
        w = line.size();
    }
    
    Map.setSize(w, h);

    // compute edges
    Map.computeEdges();

    // debug
    std::cout << "- DEBUG NODES -" << std::endl;
    Map.printNodes();

    std::cout << "\n- DEBUG EDGES -" << std::endl;
    Map.printEdges();

    std::cout << "\n- DEBUG START -" << std::endl;
    Map.printStart();

    // find largest path
    Map.Propagate();
    
    // Map.printPaths();
    std::cout << "\n- OUTPUT -" << std::endl;
    Map.printMap();
}