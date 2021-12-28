#include "led-matrix.h"
#include "graphics.h"
#include <array>
#include <cstddef>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

// TODO: Make ROWS and COLS variable, and make
// GRID_WIDTH and GRID_HEIGHT a function of
// ROWS and COLS.
#define ROWS 64
#define COLS 64
#define GRID_WIDTH 32
#define GRID_HEIGHT 32


/* -- INTERRUPT HANDLING FUNCTION --*/
volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

/* -- USER DEFINED EXCEPTIONS --*/
struct DirectionException : public std::exception
{
  const char* msg;
  DirectionException(const char* msg) {
    this->msg = msg;
  }
  const char* what () const throw ()
  {
    return this->msg;
  }
};

struct CellException : public std::exception
{
  const char* msg;
  CellException(const char* msg) {
    this->msg = msg;
  }
  const char* what () const throw ()
  {
    return this->msg;
  }
};

struct HuntNotSuccessful : public std::exception
{
  const char* what () const throw ()
  {
    return "Could not successfully find an unvisited cell";
  }
};

/*-- DIRECTION TYPING --*/
#define MAX_NUMBER_OF_NEIGHBORS 4
enum direction {Left, Up, Right, Down};
constexpr std::initializer_list<direction> all_directions = {Left, Up, Right, Down};

// define a helper function to get the opposite direction
direction getOppositeDir(direction dir) {
  if (dir == Left) {
    return Right;
  }
  else if (dir == Right) {
    return Left;
  }
  else if (dir == Up) {
    return Down;
  }
  else if (dir == Down) {
    return Up;
  }
  else {
    throw DirectionException("A direction that is not defined was attempted for use.");
  }
}

/*-- CELL DEFINITION --*/
class Cell {

  private:

    /* Member Variables */
    // x and y position of the cell (This is the center of the cell
    unsigned int matrix_x_position;
    unsigned int matrix_y_position;
    // The grid row and column position of the cell.
    unsigned int grid_row;
    unsigned int grid_column;
    // r, g, and b are the red, green, and blue color components of the cell
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    // set to true if color override is in effect
    bool color_override;
    // this array of pairs holds a reference to the neighboring cell in the
    // first item and a boolean that represents if the neighbor is connected
    // or if the neighbor is not connected.
    std::pair<Cell*, bool> neighbors[MAX_NUMBER_OF_NEIGHBORS]; 
    // this value is used to indicate if the cell in question has been visited
    // by a generation algorithm,
    bool visited;

    /* Private Function Definitions */
    // setColor is an internal function used to set the color of the cell based
    // on the current state of the cell. If the cell is connected to at least
    // one other cell, it will turn green. By default, cells are white.
    // If color override is in effect, this function takes no action.
    void setColor() {
      if (this->color_override) {
        return;
      }

      // lambda function to check if a neighbor is connected
      auto isConnected = [&] () {
        for(auto const& neighbor: this->neighbors) {
          if (neighbor.second) {
            return true;
          }
        }
        return false;
      };

      // lambda function to set the color of the cell to white
      auto setWhite = [&] () {
        this->red = 255;
        this->green = 255;
        this->blue = 255;
      };

      // lambda function to set the color of the cell to green
      auto setGreen = [&] () {
        this->red = 0;
        this->green = 255;
        this->blue = 0;
      };

      // if the cell has a connection, set green, otherwise set white.
      if (isConnected()) {setGreen();} else {setWhite();}
    }

    // function that returns true if a neighbor exists in a certain
    // direction, false if not.
    bool neighborExists(direction dir) {
      return this->neighbors[dir].first != nullptr;
    }

    // function that generates an exception if a neighbor does not
    // exist in the specified direction.
    void assertNeighborExists(direction dir) {
      if (!this->neighborExists(dir)) {
        throw CellException("There is no neigbor located in that direction.");
      }
    }

    // function that generates an exception if a neighbor exists in
    // the specified direction.
    void assertNeighborDoesNotExist(direction dir) {
      if (this->neighborExists(dir)) {
        throw CellException("A neighbor is already added at this location.");
      }
    }

  public:

    /* Public Function Definitions */
    // Constructor for a cell using the x, y position and the grid row, column position
    Cell(unsigned int matrix_x_position, unsigned int matrix_y_position, unsigned int grid_row, unsigned int grid_column) {
      this->matrix_x_position = matrix_x_position;
      this->matrix_y_position = matrix_y_position;
      this->grid_row = grid_row;
      this->grid_column = grid_column;
      for(auto & neighbor: this->neighbors) {
        neighbor = std::make_pair(nullptr, false);
      }
      this->visited = false;
      this->color_override = false;
    }

    // returns the row position of the cell in the grid
    unsigned int get_grid_row() {
      return this->grid_row;
    }

    // returns the column position of the cell in the grid
    unsigned int get_grid_column() {
      return this->grid_column;
    }

    // enforce a color override with the given r, g, b values
    void colorEnforce(unsigned int r, unsigned int g, unsigned int b) {
      this->red = r;
      this->green = g;
      this->blue = b;
      this->color_override = true;
    }

    // turn off color enforcement and let cell decide its color again
    void noColorEnforce() {
      this->color_override = false;
    }

    // This function takes a reference to a cell and it to the neighbors array at the
    // specified direction.
    void addNeighbor(direction dir, Cell *neighbor) {
      assertNeighborDoesNotExist(dir);
      this->neighbors[dir].first = neighbor;
    }

    // This function gets the reference to a neighboring cell if it exits and throws
    // an error if no neighbor exits in that direction.
    auto getNeighbor(direction dir) {
      assertNeighborExists(dir);
      return *(this->neighbors[dir].first);
    }

    // This function "turns on" a connection between two cells by setting the boolean
    // for the for the neighboring cell to true and then signaling the other cell to
    // do the same for the direction of this cell if it has not been done already.
    void connectNeighbor(direction dir) {
      assertNeighborExists(dir);
      this->neighbors[dir].second = true;

      if (!this->neighbors[dir].first->isNeighborConnected(getOppositeDir(dir))) {
        this->neighbors[dir].first->connectNeighbor(getOppositeDir(dir));
      }
    }

    // This function returns the state of the connection between the current cell and
    // the neighbor at the specified direction.
    bool isNeighborConnected(direction dir) {
      assertNeighborExists(dir);
      return this->neighbors[dir].second;
    }

    // This function sets that the cell has been visited.
    void visit() {
      this->visited = true;
    }

    // This function returns the visitation status of the this cell.
    bool isVisited() {
      return this->visited;
    }

    // Get set of valid directions that point to an actual neighbor.
    std::set<direction> getValidDirections() {
      std::set<direction> valid_directions;

      for(auto & dir: all_directions) {
        if (this->neighbors[dir].first != nullptr ) {
          valid_directions.insert(dir);
        }
      }

      if (valid_directions.empty()) {
        throw CellException("No valid direction exists from here.");
      }

      return valid_directions;
    }

    // This function returns the visitation status of the neighboring cell at a 
    // given direction.
    bool neighborIsVisitedStatus(direction dir, bool visited_status) {
      assertNeighborExists(dir);
      return this->neighbors[dir].first->isVisited() == visited_status;
    }

    // Get set of valid directions to travel from here that match the provided
    // visitation status.
    std::set<direction> getValidDirectionsWithVisitedStatus(bool visited_status) {
      std::set<direction> filtered_directions;
      std::set<direction> valid_directions = this->getValidDirections();

      for(auto i = valid_directions.begin(); i != valid_directions.end(); i++) {
        if (neighborIsVisitedStatus(*i, visited_status)) {
          filtered_directions.insert(*i);
        }
      }

      if (filtered_directions.empty()) {
        const char* errstr;
        if (visited_status) {
          errstr = "No visited direction exists from here.";
        }
        else {
          errstr = "No unvisited direction exists from here.";
        }
        throw CellException(errstr);
      }

      return filtered_directions;
    }

    // This function returns true if the cell has neighbors meeting the provided
    // visitation status.
    bool hasNeighborsWithVisitedStatus(bool visited_status) {
      try {
        auto directions = getValidDirectionsWithVisitedStatus(visited_status);
        return true;
      }
      catch (CellException& e) {
	return false;
      }
    }

    // Get set of valid directions to travel from here that have the provided
    // visitation status..
    direction getRandomDirectionWithVisitedStatus(bool visited_status) {
      auto directions = this->getValidDirectionsWithVisitedStatus(visited_status);
      unsigned int random_idx = rand() % directions.size();
      auto choice = std::begin(directions);
      std::advance(choice, random_idx);
      return *choice;
    }

    // In order to draw the cell, we make sure to set the cell core on, then we draw
    // a connection to each of the neighboring cells that are connected.
    void draw(rgb_matrix::Canvas* canvas) {
      setColor();
      canvas->SetPixel(this->matrix_x_position, this->matrix_y_position, this->red, this->green, this->blue);
      if (neighbors[Left].second) {
        canvas->SetPixel(this->matrix_x_position, this->matrix_y_position-1, this->red, this->green, this->blue);
      }
      if (neighbors[Up].second) {
        canvas->SetPixel(this->matrix_x_position+1, this->matrix_y_position, this->red, this->green, this->blue);
      }
      if (neighbors[Right].second) {
        canvas->SetPixel(this->matrix_x_position, this->matrix_y_position+1, this->red, this->green, this->blue);
      }
      if (neighbors[Down].second)  {
        canvas->SetPixel(this->matrix_x_position-1, this->matrix_y_position, this->red, this->green, this->blue);
      }
    }
};

/* -- GRID DEFINITION -- */
class Grid {

  private:

    /* Member Variables */
    //2D vector of shared pointers to Cells
    std::vector<std::vector<Cell>> cells;
    //reference to the RGB canvas, assumed to be freed by the caller
    rgb_matrix::Canvas *canvas;

    /* Private Function Definitions */
    // This function is used to create the cells that that compose
    // the grid.
    std::vector<std::vector<Cell>> createCells() {
      // 2D vector of Cells
      std::vector<std::vector<Cell>> cells;
      // for the height of the grid we will create a new row of cells
      for (unsigned int i = 0; i < GRID_HEIGHT; i++) {
        std::vector<Cell> row;
        // for the width of the grid we will push a new cell into the
        // row at the appropriate offset
        for (unsigned int j = 0; j < GRID_WIDTH; j++) {
          Cell c(i*2, j*2, i, j);
          // push the cell on the back of the row
          row.push_back(c);
        }
        // Now that the row is populated, add it to the pile.
        cells.push_back(row);
      }
      //we have all the rows now. send it!
      return cells;
    }

    // Once the cells have been populated in the grid, we walk each cell to make sure
    // that it knows its neighbors.
    void populateNeighbors() {
      // this lambda function can determine if a cell has a neighbor in a given direction
      // by checking the cell's location in the grid.
      auto hasNeighbor = [&] (direction dir, std::size_t row, std::size_t column) {
        switch(dir) {
          case Left:
            return column != 0;
            break;
          case Up:
            return row != GRID_HEIGHT-1;
            break;
          case Right:
            return column != GRID_WIDTH-1;
            break;
          case Down:
            return row != 0;
            break;
          default:
            throw DirectionException("A direction that is not defined was attempted for use.");
        }
      };

      // this lambda function adds the neighbors on each side of cell appropriately
      // if it is determined that it has a neighbor in that direction.
      auto addNeighborsToCell = [&] (Cell* cell, std::size_t row, std::size_t column) {
        if (hasNeighbor(Right, row, column)) 
          cell->addNeighbor(Right, &(this->cells[row][column+1]));
        if (hasNeighbor(Up, row, column)) 
          cell->addNeighbor(Up, &(this->cells[row+1][column]));
        if (hasNeighbor(Left, row, column)) 
          cell->addNeighbor(Left, &(this->cells[row][column-1]));
        if (hasNeighbor(Down, row, column)) 
          cell->addNeighbor(Down, &(this->cells[row-1][column]));
      };

      // For each cell in the grid, we add its neighbors.
      for (std::size_t i = 0; i < this->cells.size(); i++) {
        for (std::size_t j = 0; j < this->cells[i].size(); j++) {
          addNeighborsToCell(&this->cells[i][j], i, j);
        }
      }
    }

  public:

    /* Public Function Definitions */
    // Constructor for the grid creates the 2D vector of cells
    // and then populates the neighbors for each.
    Grid(rgb_matrix::Canvas *canvas) {
      this->canvas = canvas;
      this->cells = createCells();
      populateNeighbors();
    }

    // Destructor needs to clear the canvas
    ~Grid() {
      this->canvas->Fill(0, 0, 0);
    }

    // The grid is composed of all the cells, to draw the grid,
    // all we need to do is draw each of the cells.
    void draw() {
      for ( auto & row : this->cells ) {
        for ( auto & cell : row ) {
          cell.draw(this->canvas);
        }
      }
    }

    // The is an accessor function to get all cells from the
    // grid for the purposes of scanning.
    auto const getCells() {
      return this->cells;
    }

    // The is an mutator function to set a cell in the grid
    void setCell(Cell c) {
      this->cells[c.get_grid_row()][c.get_grid_column()] = c;
    }

    // The is an accessor function to get a cell from the grid,
    // indexed by row and column.
    auto getCell(unsigned int row, unsigned int column) {
      return this->cells[row][column];
    }

    // The is an accessor function to get a cell from the grid
    // at random
    auto getRandomCell() {
      unsigned int row = rand() % (GRID_HEIGHT-1);
      unsigned int column = rand() % (GRID_WIDTH-1);
      return this->cells[row][column];
    }
};

/* -- STRATEGY FUNCTIONS FOR MAZE GENERATION -- */
// Hunt and kill strategy which takes a grid object
// and generates a Maze
void huntAndKill(Grid* g) {
  auto walk = [&] (Cell cell) {
    // create current cell from passed in cell
    Cell current_cell = cell;

    // while the current cell in question does not have unvisited neighbors.
    while (current_cell.hasNeighborsWithVisitedStatus(false)) {
      if (interrupt_received) {
        return;
      }

      // decide the next direction at random from the unvisited cells around this one.
      direction dir = current_cell.getRandomDirectionWithVisitedStatus(false);
      //
      // connect current cell to neighbor and update it
      current_cell.connectNeighbor(dir);
      g->setCell(current_cell);
      // move current cell to neighbor and update it
      current_cell = current_cell.getNeighbor(dir);
      current_cell.visit();
      g->setCell(current_cell);

      // draw the grid for the update in walk
      g->draw();
      usleep(1 * 1000);  // wait a little to slow down things.
    }
  };

  auto hunt = [&] () {
    auto findTargetCell = [&] () {
      auto cells = g->getCells();
      for (std::size_t i = 0; i < cells.size(); i++) {
        for (std::size_t j = 0; j < cells[i].size(); j++) {
          if (cells[i][j].isVisited())
            continue;
          if (!cells[i][j].hasNeighborsWithVisitedStatus(true))
            continue;
	  return g->getCell(i, j);
        }
      }
      throw HuntNotSuccessful();
    };

    // get target cell and direction by which to connect to the
    // existing maze.
    Cell c = findTargetCell();
    direction dir = c.getRandomDirectionWithVisitedStatus(true);

    // update target cell
    c.colorEnforce(255, 0, 0);
    c.visit();
    c.connectNeighbor(dir);
    g->setCell(c);

    // draw the grid to show the new target cell
    g->draw();
    usleep(1 * 1000000);  // wait a little to slow down things.

    // turn off the color override
    c.noColorEnforce();
    g->setCell(c);
    return c;

  };

  auto cell = g->getRandomCell();
  cell.visit();
  g->setCell(cell);
  while(true) {
    try {
      if (interrupt_received)
        return;
      walk(cell);
      cell = hunt();
    } 
    catch (HuntNotSuccessful& e) {
      break;
    }
  }
}

/* -- STRATEGY DISPATCH FUNCTION == */
void createMaze(rgb_matrix::Canvas* canvas) {
  while(!interrupt_received) {
    Grid g(canvas);
    huntAndKill(&g);
    usleep(10 * 1000000);  // wait a little to slow down things.
  }
};

/* -- DRIVER FUNCTION == */
int main(int argc, char **argv) {
  srand(time(NULL));

  rgb_matrix::RGBMatrix::Options led_options;
  rgb_matrix::RuntimeOptions runtime;
  // Set defaults
  led_options.hardware_mapping = "adafruit-hat";
  led_options.chain_length = 1;
  //led_options.show_refresh_rate = true;
  led_options.rows = ROWS;
  led_options.cols = COLS;
  runtime.drop_privileges = 1;
  if (!rgb_matrix::ParseOptionsFromFlags(&argc, &argv, &led_options, &runtime)) {
    rgb_matrix::PrintMatrixFlags(stderr);
    return 1;
  }
  // Looks like we're ready to start
  rgb_matrix::RGBMatrix *matrix = rgb_matrix::RGBMatrix::CreateFromOptions(led_options, runtime);
  if (matrix == NULL) {
    return 1;
  }
  rgb_matrix::Canvas *canvas = matrix;

  // register interrupts
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);

  //  .. now use matrix
  createMaze(canvas);

  //Clear the matrix and remove the resources that are used
  canvas->Clear();
  delete canvas;
  canvas = nullptr;
  matrix = nullptr;
  return 0;
}
