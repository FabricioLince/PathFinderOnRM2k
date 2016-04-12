#include <iostream>
#include <fstream>
#include <string>

#include "GridGraph.h"
#include "PathFinder/AStar.h"

#define inputFileName "channel.txt"
#define outputFileName "channel.txt"
#define responseOK "ok_response"
#define requestOK "rm_request"

#define repeat(i, t) for (int i = 0; i < t; ++i)
#define repeat_u(i, t) for (unsigned i = 0; i < t; ++i)

char* bytesFromInt(int number)
{
    static char b[4];
    repeat(i, 4)
    {
        b[i] = ((unsigned char*)(&number))[i];
    }
    return b;
}
int intFromBytes(char b[4])
{
    int number = 0;
    repeat(i, 4)
    {
        ((unsigned char*)(&number))[i] = b[i];
    }
    return number;
}

int readIntFromBytes(std::istream& file)
{
    char b[4];
    repeat(i, 4)
    {
        file.get(b[i]);
    }
    return intFromBytes(b);
}

class RMAI
{
    public:
        std::map<int , GridGraph> graph;

    public:
        RMAI()
        {}

        void sendResponse(const std::string& response)
        {
            std::fstream file(outputFileName, std::ios::out);

            file.write(response.c_str(), response.length());

            file.close();

            std::fstream okFile(responseOK, std::ios::out);
            okFile << "1";
        }

        void fixPassableCeilingTiles(GridGraph& map)
        {
            int passableCeilingTileNumber = 12;
            int resultingCost = 1;

            std::map<Coord, int>::iterator it = map.cost.begin();

            for (; it != map.cost.end(); it++)
            {
                if (it->second == passableCeilingTileNumber)
                {
                    Coord up = it->first;
                    up.y -= 1;
                    std::map<Coord, int>::iterator itUp = map.cost.find(up);
                    if (itUp != map.cost.end())
                    {
                        if (itUp->second == passableCeilingTileNumber)
                        {
                            map.passable[it->first] = false;
                            map.passable.erase(map.passable.find(it->first));
                        }
                    }
                }
            }
            it = map.cost.begin();
            for (; it != map.cost.end(); it++)
            {
                if (it->second == passableCeilingTileNumber)
                {
                    it->second = resultingCost;
                }
            }
        }
        void readCoordsFromFile(std::fstream& file, GridGraph& map, int mapHeight)
        {
            char byte;
            int x = 0, y = 0;
            while ((byte = file.get()) != EOF)
            {
                Coord coord(x, y);
                if (byte != 0 && byte != 9 && byte != 10)
                {
                    map.passable[coord] = true;
                    map.cost[coord] = byte;
                }
                else
                {
                    map.passable.erase(coord);
                    map.cost.erase(coord);
                }

                y++;
                if (y == mapHeight)
                {
                    y = 0;
                    x++;
                }
            }
        }
        int readNewMap(std::fstream& file)
        {
            int id = readIntFromBytes(file);
            std::cout << "ID: " << id << "\n";

            int w = readIntFromBytes(file);
            std::cout << "W: " << w << "\n";

            int h = readIntFromBytes(file);
            std::cout << "H: " << h << "\n";

            GridGraph& map = graph[id];

            readCoordsFromFile(file, map, h);

            fixPassableCeilingTiles(map);

            std::cout << "[\n" << map << "]\n";

            return id;
        }

        void pathToSring(std::vector<Coord>& path, std::string& stringPath)
        {
            Coord before = path.at(0);
            repeat_u(i, path.size())
            {
                if (i == 0) continue;

                Coord& now = path.at(i);
                     if (now.x > before.x) stringPath += "R";
                else if (now.x < before.x) stringPath += "L";
                else if (now.y > before.y) stringPath += "D";
                else if (now.y < before.y) stringPath += "U";

                before = now;
            }
        }
        void readCoordFromFile(std::fstream& file, Coord& start, Coord& goal)
        {
            start.x = readIntFromBytes(file);
            start.y = readIntFromBytes(file);
            goal.x = readIntFromBytes(file);
            goal.y = readIntFromBytes(file);
        }
        void searchPath(std::fstream& file)
        {
            int id = readIntFromBytes(file);
            if (graph.count(id) == 0)
            {
                std::cerr << "No graph for id: " << id << "\n";
                return;
            }
            GridGraph& map = graph[id];

            Coord start, goal;
            readCoordFromFile(file, start, goal);

            std::cout << "Start: " << start << "\nGoal: " << goal << "\n";

            std::vector<Coord> path = AStar::search(map, start, goal);
            std::string stringPath;

            if (path.size() > 0)
            {
                pathToSring(path, stringPath);
                std::cout << stringPath << "\n";

                sendResponse(stringPath);
            }
            else
            {
                std::cout << "No Path Found\n";
                sendResponse("Z");
            }

            //animateMove(map, path);
        }

        bool waitForCommand()
        {
            while (true)
            {
                std::fstream file(requestOK, std::ios::in);
                if (file.is_open()) break;
            }
            remove(requestOK);
            std::cout << "Request Received\n";

            std::fstream input(inputFileName, std::ios::in);
            if (input.is_open())
            {
                input.seekg(0);

                char command = 0;
                command = input.get();
                if (command == EOF)
                {
                    std::cerr << "No command\n";
                }
                else if (command == 'M')
                {
                    std::cout << "New Map\n";
                    readNewMap(input);
                }
                else if (command == 'S')
                {
                    std::cout << "Search Path\n";
                    searchPath(input);
                }
                else
                {
                    std::cerr << "Command not recognized: "
                    << command << "(" << (int)command << ")\n";
                    return false;
                }

                input.close();
            }
            else
            {
                std::cerr << "couldn't open channel...\n";
                //return false;
            }

            return true;
        }

        void run()
        {
            std::cout << "running\n";

            while (waitForCommand());

            std::cout << "done\n";
        }
};

int main(int argc, char** argv)
{
    try
    {
        RMAI ai;
        ai.run();

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception\n" << e.what() << "\n";
    }
    return 1;
}
