// path-finding.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <random>
#include <queue>
#include <algorithm>
#include <cmath>
#include<climits>
struct Point {
    int row; int column;
};
class Grid {
    private:
        int rows_;
        int cols_;
        std::vector<char> cells_;
    public:
        Grid(int rows, int cols, double obstacleRatio):rows_(rows),cols_(cols),cells_(rows*cols, '.')
            {
                std::mt19937 rng(42);
                std::bernoulli_distribution wall(obstacleRatio);
                for (size_t i{0};i<cells_.size();++i) {
                    if(wall(rng))cells_[i]='#';
                }
            }
        void print() const {
            for (size_t i{ 0 }; i < rows_; ++i) {
                for (size_t j{ 0 }; j < cols_; ++j) {
                    std::cout<<cells_[i*cols_+j];
                }
                std::cout<<std::endl;
            }
        }
        bool inBound(Point p) const {
            return p.row>=0&&p.row<rows_&&p.column>=0&&p.column<cols_;
        }
        bool free(Point p) const {
            return inBound(p)&&cells_[p.row*cols_+p.column]!='#';
        }
        char& at(Point p) {
            return cells_[p.row*cols_+p.column];
        }
        int rows() const{return rows_;}
        int cols() const{return cols_;}
        std::vector<Point> neighbors(Point p) const {
            std::vector<Point> out;
            Point pUp{p.row-1,p.column}, pDown{p.row+1,p.column}, pLeft{p.row,p.column-1}, pRight{p.row,p.column+1};
            if(free(pUp))out.push_back(pUp);if (free(pDown))out.push_back(pDown);if (free(pLeft))out.push_back(pLeft);if (free(pRight))out.push_back(pRight);
            return out;
        }
};

std::vector<Point> bfs(const Grid& g, Point start, Point goal) {
    int n=g.rows() * g.cols();
    std::vector<int> parent(n,-1);
    std::vector<bool> seen(n,false);
    std::queue<Point> q;
    auto id = [&](Point p) {return p.row * g.cols() + p.column;};
    int steps{0};
    q.push(start);
    seen[id(start)]=true;
    while (!q.empty()){
        if(q.front().column==goal.column&&q.front().row==goal.row) break;
        else {
            Point cur = q.front();q.pop();
            ++steps;
            for (Point p : g.neighbors(cur)) {
                if(!seen[id(p)]){
                    seen[id(p)]=true;
                    parent[id(p)]=id(cur);
                    q.push(p);
                    }
                }
            }
    }
    std::cout << "Nodes expanded (BFS): " << steps << std::endl;
    if(!seen[id(goal)]) return std::vector<Point>{};
    std::vector<Point> path;
    int temp{id(goal)};
    while (temp >= 0) {
       path.push_back({ temp / g.cols(),temp % g.cols() });
       temp = parent[temp];
    }
    std::reverse(path.begin(),path.end());
    return path;
}
std::vector<Point> astar(const Grid& g, Point start, Point goal) {
    int n = g.rows() * g.cols();
    std::vector<int> parent(n, -1);
    std::vector<int> gScore(n, INT_MAX);
    std::vector<bool> closed(n, false);
    auto id = [&](Point p) {return p.row * g.cols() + p.column;};
    auto h = [&](Point p) {return std::abs(p.row - goal.row) + std::abs(p.column - goal.column);};
    int steps{ 0 };
    using Node = std::pair<int, int>;   // (f, cell id)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    gScore[id(start)] = 0;
    open.push({ h(start), id(start) });

    while (!open.empty()) {
        int curId = open.top().second; open.pop();
        if (closed[curId]) continue;
        closed[curId] = true;
        ++steps;
        Point cur{ curId / g.cols(), curId % g.cols() };
        if (cur.column == goal.column && cur.row == goal.row) break;

        for (Point p : g.neighbors(cur)) {
            int temp = gScore[curId] + 1;
            if (temp < gScore[id(p)]) {
                gScore[id(p)] = temp;
                parent[id(p)] = curId;
                open.push({ temp + h(p), id(p) });
            }
        }
    }
    std::cout << "Nodes expanded (A*): " << steps << std::endl;
    if (!closed[id(goal)]) return std::vector<Point>{};
    std::vector<Point> path;
    int temp{ id(goal) };
    while (temp >= 0) {
        path.push_back({ temp / g.cols(),temp % g.cols() });
        temp = parent[temp];
    }
    std::reverse(path.begin(), path.end());
    return path;
}
        
    
int main()
{
    Grid g1(15,40,0.25);
    std::cout<<"Original grid: "<<std::endl;
    g1.print();
    std::cout<<"\n\n#################################################\n\n"<<std::endl;
    Point start{0,0};
    Point goal{14,39};
    g1.at(start)='.'; g1.at(goal) = '.';
    std::cout << "BFS traversal: " << std::endl;
    std::vector<Point> path{ bfs(g1, start, goal) };
    for(Point p : path) {
        g1.at(p)='*';
    }
    g1.at(start)='S';g1.at(goal)='G';
    g1.print();
    if (path.empty()) std::cout << "No path"<<std::endl;
    else std::cout <<"\nPath length: "<<path.size() - 1 <<" steps"<<std::endl;
    std::cout << "\n\n#################################################\n\n" << std::endl;
    Grid g2(15,40,0.25);
    g2.at(start) = '.'; g2.at(goal) = '.';
    std::cout << "A* traversal: " << std::endl;
    std::vector<Point> path2{ astar(g2, start, goal) };
    for (Point p : path2) {
        g2.at(p) = '*';
    }
    g2.at(start) = 'S';g2.at(goal) = 'G';
    g2.print();
    if (path2.empty()) std::cout << "No path" << std::endl;
    else std::cout <<"\nPath length: "<<path2.size() - 1 <<" steps"<< std::endl;
}