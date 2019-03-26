/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <FastLED.h>

#ifdef __AVR__

#include <avr/power.h>
#include <tetris/consts.h>

#endif
#define GRID_W 9
#define GRID_H 16
#define BLOCK_SIZE_IN_PX 4
#define BLOCK_ROTATIONS 4
#define BLOCK_TYPES 7

int max_x,
    max_y;

enum Rotation
{
    r0,
    r90,
    r180,
    r270
};

enum BlockTypes
{
    T,
    O,
    I,
    J,
    L,
    S,
    Z
};

struct Point
{
    int x;
    int y;
};

int block_rotation = r0;
Point block_position = {4, 14};
int block_type = L;
Point spawn_point_position = Point{4, 14};
int y_row = 1;
int color;
bool start_game = false;
Point blocks[BLOCK_TYPES][BLOCK_ROTATIONS][BLOCK_SIZE_IN_PX] =
    {
        // block T
        Point{0, 0},
        Point{1, 0},
        Point{-1, 0},
        Point{0, 1},

        Point{0, 0},
        Point{0, 1},
        Point{1, 0},
        Point{0, -1},

        Point{0, 0},
        Point{-1, 0},
        Point{1, 0},
        Point{0, -1},

        Point{0, 0},
        Point{0, -1},
        Point{-1, 0},
        Point{0, 1},

        // block O
        Point{0, 0},
        Point{1, 0},
        Point{0, -1},
        Point{1, -1},

        Point{0, 0},
        Point{1, 0},
        Point{0, -1},
        Point{1, -1},

        Point{0, 0},
        Point{1, 0},
        Point{0, -1},
        Point{1, -1},

        Point{0, 0},
        Point{1, 0},
        Point{0, -1},
        Point{1, -1},

        // block I

        Point{-2, 0},
        Point{-1, 0},
        Point{0, 0},
        Point{1, 0},

        Point{0, 1},
        Point{0, 0},
        Point{0, -1},
        Point{0, -2},

        Point{-2, -1},
        Point{-1, -1},
        Point{0, -1},
        Point{1, -1},

        Point{-1, 1},
        Point{-1, 0},
        Point{-1, -1},
        Point{-1, -2},

        // block J

        Point{-1, 1},
        Point{-1, 0},
        Point{0, 0},
        Point{1, 0},

        Point{0, -1},
        Point{0, 0},
        Point{0, 1},
        Point{1, 1},

        Point{-1, 0},
        Point{0, 0},
        Point{1, 0},
        Point{1, -1},

        Point{0, 1},
        Point{0, 0},
        Point{0, -1},
        Point{-1, -1},

        // block L

        Point{-1, 0},
        Point{0, 0},
        Point{1, 0},
        Point{1, 1},

        Point{0, 1},
        Point{0, 0},
        Point{0, -1},
        Point{1, -1},

        Point{-1, -1},
        Point{-1, 0},
        Point{0, 0},
        Point{1, 0},

        Point{-1, 1},
        Point{0, 1},
        Point{0, 0},
        Point{0, -1},

        // block S
        Point{0, 0},
        Point{0, 1},
        Point{1, 1},
        Point{-1, 0},

        Point{0, 1},
        Point{0, 0},
        Point{1, 0},
        Point{1, -1},

        Point{0, 0},
        Point{0, -1},
        Point{-1, -1},
        Point{1, 0},

        Point{-1, 1},
        Point{-1, 0},
        Point{0, 0},
        Point{0, -1},

        // block Z

        Point{-1, 1},
        Point{0, 1},
        Point{0, 0},
        Point{1, 0},

        Point{0, -1},
        Point{0, 0},
        Point{1, 0},
        Point{1, 1},

        Point{-1, 0},
        Point{0, 0},
        Point{0, -1},
        Point{1, -1},

        Point{-1, -1},
        Point{-1, 0},
        Point{0, 0},
        Point{0, 1}};

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

const int readPin = 32; // Use GPIO number. See ESP32 board pinouts
const int LED = 2;      // Could be different depending on the dev board. I used the DOIT ESP32 dev board.
bool force_game_finished = false;

typedef struct SEMAPHORE_PARAMETERS
{
    xSemaphoreHandle xSemaphore;
    uint32_t dx = 0;
    uint32_t r = 255;
    uint32_t g = 0;
    uint32_t b = 0;
} xSemaphoreParameters;

xSemaphoreParameters _g_parameters;

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define PIN 22
#define NUMPIXELS 144
CRGB leds[NUMPIXELS];
bool grid[GRID_W][GRID_H];
static int start_values[] = {0, 31, 32, 63, 64, 95, 96, 127, 128};

int dx = 0;
Point block[4];

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

void displayLed(int x, int y, int r, int g, int b)
{
    if (x % 2 == 0)
    {
        leds[start_values[x] + y] = CRGB(r, g, b);
    }
    else
    {
        leds[start_values[x] - y] = CRGB(r, g, b);
    }
}

void drawPixel(int x, int y, int color)
{
    if(color == 1)
        {
            displayLed(x, y, 161, 1, 244);
        }
        else if (color == 2)
        {
            displayLed(x, y, 240, 241, 0);
        }
        else if (color == 3)
        {
            displayLed(x, y, 48, 199, 239);
        }
        else if (color == 4)
        {
            displayLed(x, y, 239, 161, 0);
        }
        else if (color == 5)
        {
            displayLed(x, y, 0, 2, 236);
        }
        else if (color == 6)
        {
            displayLed(x, y, 0, 255, 0);
        }
        else if (color == 7)
        {
            displayLed(x, y, 255, 0, 0);
        }
}

void clearPixel(int x, int y)
{
    displayLed(x, y, 0, 0, 0);
}

void draw_block()
{
    if (block_type == T)
    {
        color = 1;
    }
    else if (block_type == O)
    {
        color = 2;
    }
    else if (block_type == I)
    {
        color = 3;
    }
    else if (block_type == J)
    {
        color = 4;
    }
    else if (block_type == L)
    {
        color = 5;
    }
    else if (block_type == S)
    {
        color = 6;
    }
    else if (block_type == Z)
    {
        color = 7;
    }
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        drawPixel(block[i].x, block[i].y,color);
    }
}

void clear_block()
{
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        clearPixel(block[i].x, block[i].y);
    }
}

void refresh_block_pixels()
{
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        block[i].x = block_position.x + blocks[block_type][block_rotation][i].x;
        block[i].y = block_position.y + blocks[block_type][block_rotation][i].y;
    }
}

void randomize_block()
{
    block_type = BlockTypes(rand() % 7);
}

void lock_block()
{
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        grid[block[i].x][block[i].y] = true;
    }
}

bool is_row_full(int y)
{
    for (int x = 0; x < GRID_W; x++)
    {
        if (!grid[x][y])
        {
            return false;
        }
    }

    return true;
}

void clear_row(int y_row)
{
    for (int x = 0; x < GRID_W; x++)
    {
        clearPixel(x, y_row);
        grid[x][y_row] = false;
    }
}

void move_row_down(int y_row, int offset)
{
    for (int x = 0; x < GRID_W; x++)
    {
        grid[x][y_row - offset] = grid[x][y_row];
        if (grid[x][y_row])
        {
            drawPixel(x, y_row - offset,color);
        }
        else
        {
            clearPixel(x, y_row - offset);
        }

        clearPixel(x, y_row);
    }
}

void clear_full_rows()
{
    int number_of_full_rows_found = 0;
    int first_full_row_y = 0;
    for (int y = 0; y < GRID_H; y++)
    {
        if (is_row_full(y))
        {
            if (number_of_full_rows_found == 0)
            {
                first_full_row_y = y;
            }

            number_of_full_rows_found++;
            clear_row(y);
        }
        else
        {
            if (number_of_full_rows_found > 0)
            {
                move_row_down(y, number_of_full_rows_found);
            }
        }
    }
}

int getNextRotation()
{
    int new_rotation = block_rotation + 1;
    return new_rotation < 4 ? new_rotation : 0;
}

void try_to_rotate(void *parameter)
{
    int targetRotation = getNextRotation();
    Serial.print(targetRotation);
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        int pixel_x = block_position.x + blocks[block_type][targetRotation][i].x;
        int pixel_y = block_position.y + blocks[block_type][targetRotation][i].y;

        // check boundary conditions
        if (pixel_x > max_x || pixel_x < 0 || pixel_y > max_y || pixel_y < 0)
        {
            vTaskDelete(NULL);
            return;
        }

        // check collisions with existing pixels
        if (grid[pixel_x][pixel_y])
        {
            vTaskDelete(NULL);
            return;
        }
    }
    Serial.println(" OK");
    clear_block();
    block_rotation = targetRotation;
    refresh_block_pixels();
    draw_block();
    vTaskDelete(NULL);
}

void try_to_move_x(void *parameter)
{
    int dx = _g_parameters.dx;
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        if ((block[i].x == 0 && dx == -1) || (block[i].x == 8 && dx == 1)) //detects if block is meeting grid' walls
        {
            vTaskDelete(NULL);
            return;
        }
        if (grid[block[i].x - 1][block[i].y] && dx == -1 || grid[block[i].x + 1][block[i].y] && dx == 1) // detects if block is meeting other block' side walls
        {
            vTaskDelete(NULL);
            return;
        }
    }
    block_position.x += dx;
    Serial.print("x : ");
    Serial.println(block_position.x);
    vTaskDelete(NULL);
}

bool collision()
{
    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        if (block[i].y == 0 || grid[block[i].x][block[i].y - 1]) // any block reached bottommost row OR there is other block in the grid below it
        {
            return true;
        }
    }

    return false;
}

// void dump_grid()
// {
//     for (int i = 15; i >= 0; i--)
//     {
//         for (int j = 0; j < 9; j++)
//         {
//             Serial.print(grid[j][i] ? "X" : "_");
//             Serial.print("");
//         }
//         Serial.println();
//     }
// }

void clear_grid()
{
    for (int i = 0; i < GRID_W; i++)
    {
        for (int j = 0; j < GRID_H; j++)
        {
            grid[i][j] = false;
        }
    }
}

void rendering()
{
    delay(100);
    FastLED.show();
}

void update_colors(int r, int g, int b)
{
    _g_parameters.r = r;
    _g_parameters.g = g;
    _g_parameters.b = b;
}

void randomize_color()
{
    update_colors(rand() % 255, rand() % 255, rand() % 255);
}

bool game_finished()
{
    if (force_game_finished)
    {
        return true;
    }

    for (int i = 0; i < BLOCK_SIZE_IN_PX; i++)
    {
        if (grid[block[i].x][block[i].y])
        {
            return true;
        }
    }

    return false;
}

void gameLoop()
{
    while (true)
    {
        FastLED.clear();
        clear_grid();
        block_position = {4, 14};
        randomize_block();
        refresh_block_pixels();
        force_game_finished = false;

        while (!game_finished())
        {
            delay(500);

            if (!collision())
            {
                // Serial.println("falling...");
                clear_block();
                block_position.y--;
                refresh_block_pixels();
                draw_block();
                rendering();
            }
            else
            {
                Serial.println("RESPAWN");
                lock_block();
                clear_full_rows();
                // dump_grid();
                block_position = spawn_point_position;
                randomize_block();
                refresh_block_pixels();
                draw_block();
                rendering();
            }
        }

        force_game_finished = false;
        // screen_is_full = false;
        while(!force_game_finished)
        {
            for (int dot = 0; dot < NUMPIXELS; dot++)
            {
                leds[dot] = CRGB::Red;
                FastLED.show();
                delay(40);
                // will sit here until restart is pressed
            }
        }
    }

    vTaskDelete(NULL);
}

void update_move_param(int dx)
{
    _g_parameters.dx = dx;
}

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();

        /* xTaskCreatePinnedToCore(
            coreTask,   Function to implement the task 
            "coreTask", Name of the task 
            10000,      Stack size in words
            NULL,       Task input parameter
            0,          Priority of the task
            NULL,       Task handle. 
            taskCore);  Core where the task should run */

        if (rxValue.find("left") != -1)
        {
            Serial.print("left");
            update_move_param(-1);
            xTaskCreatePinnedToCore(try_to_move_x, "try_to_move_x", 10000, NULL, 1, NULL, 0);
        }
        else if (rxValue.find("right") != -1)
        {
            Serial.print("right");
            update_move_param(1);
            xTaskCreatePinnedToCore(try_to_move_x, "try_to_move_x", 10000, NULL, 1, NULL, 0);
        }
        else if (rxValue.find("rotate") != -1)
        {
            Serial.print("rotate");
            xTaskCreatePinnedToCore(try_to_rotate, "try_to_rotate", 10000, NULL, 1, NULL, 0);
        }
        else if (rxValue.find("restart") != -1)
        {
            Serial.print("restart");
            start_game = true;
            force_game_finished = true;
        }
    }
};

void setup()
{
    max_x = GRID_W - 1;
    max_y = GRID_H - 1;
    FastLED.addLeds<NEOPIXEL, PIN>(leds, NUMPIXELS);
    Serial.begin(115200);
    FastLED.clear();
    static int taskCore = 0;
    // Create the BLE Device
    BLEDevice::init("ESP32 Tetris"); // Give it a name

    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();

    Serial.println("Waiting a client connection to notify...");

    refresh_block_pixels();

}

void loop() {
    if (start_game)
    {
        gameLoop();
    }
}
