//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include "gui/imgui.h"
#include "gui/utils.h"

#ifdef WIN32
# define snprintf _snprintf
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const unsigned TEXT_POOL_SIZE = 8000;
static char g_textPool[TEXT_POOL_SIZE];
static unsigned g_textPoolSize = 0;
static const char* allocText(const char* text)
{
  unsigned len = strlen(text)+1;
  if (g_textPoolSize + len >= TEXT_POOL_SIZE)
    return 0;
  char* dst = &g_textPool[g_textPoolSize]; 
  memcpy(dst, text, len);
  g_textPoolSize += len;
  return dst;
}

static const unsigned GFXCMD_QUEUE_SIZE = 1024;
static imguiGfxCmd g_gfxCmdQueue[GFXCMD_QUEUE_SIZE];
static unsigned g_gfxCmdQueueSize = 0;

static void resetGfxCmdQueue()
{
  g_gfxCmdQueueSize = 0;
  g_textPoolSize = 0;
}

static void addGfxCmdScissor(int x, int y, int w, int h)
{
  if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
    return;
  imguiGfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
  cmd.type = IMGUI_GFXCMD_SCISSOR;
  cmd.flags = x < 0 ? 0 : 1;  // on/off flag.
  cmd.col = 0;
  cmd.rect.x = (short)x;
  cmd.rect.y = (short)y;
  cmd.rect.w = (short)w;
  cmd.rect.h = (short)h;
}

static void addGfxCmdRect(int x, int y, int w, int h, unsigned int color)
{
  if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
    return;
  imguiGfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
  cmd.type = IMGUI_GFXCMD_RECT;
  cmd.flags = 0;
  cmd.col = color;
  cmd.rect.x = (short)x;
  cmd.rect.y = (short)y;
  cmd.rect.w = (short)w;
  cmd.rect.h = (short)h;
  cmd.rect.r = 0;
}

static void addGfxCmdRoundedRect(int x, int y, int w, int h, int r, unsigned int color)
{
  if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
    return;
  imguiGfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
  cmd.type = IMGUI_GFXCMD_RECT;
  cmd.flags = 0;
  cmd.col = color;
  cmd.rect.x = (short)x;
  cmd.rect.y = (short)y;
  cmd.rect.w = (short)w;
  cmd.rect.h = (short)h;
  cmd.rect.r = (short)r;
}

static void addGfxCmdTriangle(int x, int y, int w, int h, int flags, unsigned int color)
{
  if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
    return;
  imguiGfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
  cmd.type = IMGUI_GFXCMD_TRIANGLE;
  cmd.flags = (char)flags;
  cmd.col = color;
  cmd.rect.x = (short)x;
  cmd.rect.y = (short)y;
  cmd.rect.w = (short)w;
  cmd.rect.h = (short)h;
}

static void addGfxCmdText(int x, int y, int align, const char* text, unsigned int color)
{
  if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
    return;
  imguiGfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
  cmd.type = IMGUI_GFXCMD_TEXT;
  cmd.flags = 0;
  cmd.col = color;
  cmd.text.x = (short)x;
  cmd.text.y = (short)y;
  cmd.text.align = (short)align;
  cmd.text.text = allocText(text);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct GuiState
{
  GuiState() :
    left(false), leftPressed(false), leftReleased(false),
    mx(-1), my(-1), scroll(0),
    active(0), hot(0), hotToBe(0), isHot(false), isActive(false), wentActive(false),
    dragX(0), dragY(0), dragOrig(0), widgetX(0), widgetY(0), widgetW(100),
    insideCurrentScroll(false),  areaId(0), widgetId(0)
  {
  }

  bool left;
  bool leftPressed, leftReleased;
  int mx,my;
  int scroll;
  unsigned int active;
  unsigned int hot;
  unsigned int hotToBe;
  bool isHot;
  bool isActive;
  bool wentActive;
  int dragX, dragY;
  float dragOrig;
  int widgetX, widgetY, widgetW;
  bool insideCurrentScroll;
  
  unsigned int areaId;
  unsigned int widgetId;
};

static GuiState g_state;

inline bool anyActive()
{
  return g_state.active != 0;
}

inline bool isActive(unsigned int id)
{
  return g_state.active == id;
}

inline bool isHot(unsigned int id)
{
  return g_state.hot == id;
}

inline bool inRect(int x, int y, int w, int h, bool checkScroll = true)
{
   return (!checkScroll || g_state.insideCurrentScroll) && g_state.mx >= x && g_state.mx <= x+w && g_state.my >= y && g_state.my <= y+h;
}

inline void clearInput()
{
  g_state.leftPressed = false;
  g_state.leftReleased = false;
  g_state.scroll = 0;
}

inline void clearActive()
{
  g_state.active = 0;
  // mark all UI for this frame as processed
  clearInput();
}

inline void setActive(unsigned int id)
{
  g_state.active = id;
  g_state.wentActive = true;
}

inline void setHot(unsigned int id)
{
   g_state.hotToBe = id;
}


static bool buttonLogic(unsigned int id, bool over)
{
  bool res = false;
  // process down
  if (!anyActive())
  {
    if (over)
      setHot(id);
    if (isHot(id) && g_state.leftPressed)
      setActive(id);
  }

  // if button is active, then react on left up
  if (isActive(id))
  {
    g_state.isActive = true;
    if (over)
      setHot(id);
    if (g_state.leftReleased)
    {
      if (isHot(id))
        res = true;
      clearActive();
    }
  }

  if (isHot(id))
    g_state.isHot = true;

  return res;
}

static void updateInput(int mx, int my, unsigned char mbut, int scroll)
{
  bool left = (mbut & IMGUI_MBUT_LEFT) != 0;

  g_state.mx = mx;
  g_state.my = my;
  g_state.leftPressed = !g_state.left && left;
  g_state.leftReleased = g_state.left && !left;
  g_state.left = left;

  g_state.scroll = scroll;
}

void imguiBeginFrame(int mx, int my, unsigned char mbut, int scroll)
{
  updateInput(mx,my,mbut,scroll);

  g_state.hot = g_state.hotToBe;
  g_state.hotToBe = 0;

  g_state.wentActive = false;
  g_state.isActive = false;
  g_state.isHot = false;

  g_state.widgetX = 0;
  g_state.widgetY = 0;
  g_state.widgetW = 0;

  g_state.areaId = 1;
  g_state.widgetId = 1;

  resetGfxCmdQueue();
}

void imguiEndFrame()
{
  clearInput();
}

const imguiGfxCmd* imguiGetRenderQueue()
{
  return g_gfxCmdQueue;
}

int imguiGetRenderQueueSize()
{
  return g_gfxCmdQueueSize;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int BUTTON_HEIGHT = 20;
static const int SLIDER_HEIGHT = 20;
static const int SLIDER_MARKER_WIDTH = 10;
static const int CHECK_SIZE = 8;
static const int DEFAULT_SPACING = 4;
static const int TEXT_HEIGHT = 8;
static const int SCROLL_AREA_PADDING = 6;
static const int INTEND_SIZE = 16;
static const int AREA_HEADER = 28;
static const int PROGRESS_WIDTH = 280;

static int g_scrollTop = 0;
static int g_scrollBottom = 0;
static int g_scrollRight = 0;
static int g_scrollAreaTop = 0;
static int* g_scrollVal = 0;
static int g_focusTop = 0;
static int g_focusBottom = 0;
static unsigned int g_scrollId = 0;
static bool g_insideScrollArea = false;

bool imguiBeginScrollArea(const char* name, int x, int y, int w, int h, int* scroll)
{
  g_state.areaId++;
  g_state.widgetId = 0;
  g_scrollId = (g_state.areaId<<16) | g_state.widgetId;

  g_state.widgetX = x + SCROLL_AREA_PADDING;
  g_state.widgetY = y+h-AREA_HEADER + (*scroll);
  g_state.widgetW = w - SCROLL_AREA_PADDING*4;
  g_scrollTop = y-AREA_HEADER+h;
  g_scrollBottom = y+SCROLL_AREA_PADDING;
  g_scrollRight = x+w - SCROLL_AREA_PADDING*3;
  g_scrollVal = scroll;

  g_scrollAreaTop = g_state.widgetY;

  g_focusTop = y-AREA_HEADER;
  g_focusBottom = y-AREA_HEADER+h;

  g_insideScrollArea = inRect(x, y, w, h, false);
  g_state.insideCurrentScroll = g_insideScrollArea;

  addGfxCmdRoundedRect(x, y, w, h, DEFAULT_AREA_CORNER_ROUNDING, imguiRGBA(DEFAULT_SCROLLAREA_BACK_COLOR));

  addGfxCmdText(x+AREA_HEADER/2, y+h-AREA_HEADER/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, name, imguiRGBA(DEFAULT_SCROLLAREA_TEXT_COLOR));

  addGfxCmdScissor(x+SCROLL_AREA_PADDING, y+SCROLL_AREA_PADDING, w-SCROLL_AREA_PADDING*4, h-AREA_HEADER-SCROLL_AREA_PADDING);

  return g_insideScrollArea;
}

void imguiEndScrollArea()
{
  // Disable scissoring.
  addGfxCmdScissor(-1,-1,-1,-1);

  // Draw scroll bar
  int x = g_scrollRight+SCROLL_AREA_PADDING/2;
  int y = g_scrollBottom;
  int w = SCROLL_AREA_PADDING*2;
  int h = g_scrollTop - g_scrollBottom;

  int stop = g_scrollAreaTop;
  int sbot = g_state.widgetY;
  int sh = stop - sbot; // The scrollable area height.

  float barHeight = (float)h/(float)sh;
  
  if (barHeight < 1)
  {
    float barY = (float)(y - sbot)/(float)sh;
    if (barY < 0) barY = 0;
    if (barY > 1) barY = 1;
    
    // Handle scroll bar logic.
    unsigned int hid = g_scrollId;
    int hx = x;
    int hy = y + static_cast<int>(barY*static_cast<float>(h));
    int hw = w;
    int hh = static_cast<int>(barHeight*static_cast<float>(h));
    
    const int range = h - (hh-1);
    bool over = inRect(hx, hy, hw, hh);
    buttonLogic(hid, over);
    if (isActive(hid))
    {
      float u = (float)(hy-y) / (float)range;
      if (g_state.wentActive)
      {
        g_state.dragY = g_state.my;
        g_state.dragOrig = u;
      }
      if (g_state.dragY != g_state.my)
      {
        u = g_state.dragOrig + (float)(g_state.my - g_state.dragY) / (float)range;
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        *g_scrollVal = (int)((1-u) * (float)(sh - h));
      }
    }
    
    // BG
    addGfxCmdRoundedRect(x, y, w, h, /*w/2-1*/DEFAULT_AREA_CORNER_ROUNDING, imguiRGBA(DEFAULT_SCROLLAREA_BG_COLOR));
    // Bar
    if (isActive(hid))
      addGfxCmdRoundedRect(hx, hy, hw, hh, /*w/2-1*/DEFAULT_AREA_CORNER_ROUNDING, imguiRGBA(ACTIVE_SCROLLAREA_BAR_COLOR));
    else
      addGfxCmdRoundedRect(hx, hy, hw, hh, /*w/2-1*/DEFAULT_AREA_CORNER_ROUNDING, isHot(hid) ? imguiRGBA(HOT_SCROLLAREA_BAR_COLOR) : imguiRGBA(DEFAULT_SCROLLAREA_BAR_COLOR));

    // Handle mouse scrolling.
    if (g_insideScrollArea) // && !anyActive())
    {
      if (g_state.scroll)
      {
        *g_scrollVal += 20*g_state.scroll;
        if (*g_scrollVal < 0) *g_scrollVal = 0;
        if (*g_scrollVal > (sh - h)) *g_scrollVal = (sh - h);
      }
    }
  }
  g_state.insideCurrentScroll = false;
}


void imguiBeginSimpleArea(const char* name, int x, int y, int w, int h)
{
  g_state.areaId++;
  g_state.widgetId = 0;

  g_state.widgetX = x + SCROLL_AREA_PADDING;
  g_state.widgetY = y+h-AREA_HEADER;
  g_state.widgetW = w - SCROLL_AREA_PADDING*4;

  g_scrollAreaTop = g_state.widgetY;

  g_focusTop = y-AREA_HEADER;
  g_focusBottom = y-AREA_HEADER+h;


  addGfxCmdRoundedRect(x, y, w, h, DEFAULT_AREA_CORNER_ROUNDING, imguiRGBA(DEFAULT_SIMPLEAREA_BACK_COLOR));

  addGfxCmdText(x+AREA_HEADER/2, y+h-AREA_HEADER/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, name, imguiRGBA(DEFAULT_SIMPLEAREA_TEXT_COLOR));

  addGfxCmdScissor(x+SCROLL_AREA_PADDING, y+SCROLL_AREA_PADDING, w-SCROLL_AREA_PADDING, h-AREA_HEADER-SCROLL_AREA_PADDING);

}

void imguiEndSimpleArea()
{
  // Disable scissoring.
  addGfxCmdScissor(-1,-1,-1,-1);

  // Draw scroll bar
  int x = g_scrollRight+SCROLL_AREA_PADDING/2;
  int y = g_scrollBottom;
  int w = SCROLL_AREA_PADDING*2;
  int h = g_scrollTop - g_scrollBottom;

  // BG
  addGfxCmdRoundedRect(x, y, w, h, /*w/2-1*/DEFAULT_AREA_CORNER_ROUNDING, imguiRGBA(DEFAULT_SIMPLEAREA_BG_COLOR));
  
}
bool imguiButton(const char* text, bool enabled)
{
  g_state.widgetId++;
  unsigned int id = (g_state.areaId<<16) | g_state.widgetId;
  
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  int w = g_state.widgetW;
  int h = BUTTON_HEIGHT;
  g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

  bool over = enabled && inRect(x, y, w, h);
  bool res = buttonLogic(id, over);

  addGfxCmdRoundedRect(x, y, w, h, /*BUTTON_HEIGHT/2-1*/DEFAULT_BUTTON_CORNER_ROUNDING, isActive(id)? imguiRGBA(ACTIVE_BUTTON_COLOR) : imguiRGBA(DEFAULT_BUTTON_COLOR));
  if (enabled)
    addGfxCmdText(x+BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, isHot(id) ? imguiRGBA(HOT_BUTTON_TEXT_COLOR) : imguiRGBA(DEFAULT_BUTTON_TEXT_COLOR));
  else
    addGfxCmdText(x+BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(DISABLED_BUTTON_TEXT_COLOR));

  return res;
}

bool imguiItem(const char* text, bool selected, bool enabled)
{
  g_state.widgetId++;
  unsigned int id = (g_state.areaId<<16) | g_state.widgetId;
  
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  int w = g_state.widgetW;
  int h = BUTTON_HEIGHT;
  g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;
  
  bool over = enabled && inRect(x, y, w, h);
  bool res = buttonLogic(id, over);
  
  if (selected)
    addGfxCmdRoundedRect(x, y, w, h, 2, imguiRGBA(SELECTED_ITEM_COLOR));
  else if (isHot(id)) 
    addGfxCmdRoundedRect(x, y, w, h, 2, isActive(id) ? imguiRGBA(ACTIVE_ITEM_COLOR) : imguiRGBA(SELECTED_ITEM_COLOR));

  if (enabled)
    addGfxCmdText(x+BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(DEFAULT_ITEM_TEXT_COLOR));
  else
    addGfxCmdText(x+BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(DISABLED_ITEM_TEXT_COLOR));
  
  return res;
}


bool imguiCheck(const char* text, bool checked, bool enabled)
{
  g_state.widgetId++;
  unsigned int id = (g_state.areaId<<16) | g_state.widgetId;
  
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  int w = g_state.widgetW;
  int h = BUTTON_HEIGHT;
  g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

  bool over = enabled && inRect(x, y, w, h);
  bool res = buttonLogic(id, over);
  
  const int cx = x+BUTTON_HEIGHT/2-CHECK_SIZE/2;
  const int cy = y+BUTTON_HEIGHT/2-CHECK_SIZE/2;
  addGfxCmdRoundedRect(cx-3, cy-3, CHECK_SIZE+6, CHECK_SIZE+6, DEFAULT_CHECK_CORNER_ROUNDING, isActive(id)? imguiRGBA(ACTIVE_CHECK_COLOR) : imguiRGBA(DEFAULT_CHECK_COLOR) );
  if (checked)
  {
    if (enabled)
      addGfxCmdRoundedRect(cx, cy, CHECK_SIZE, CHECK_SIZE, CHECK_SIZE/2-1/*DEFAULT_CHECK_CORNER_ROUNDING*/, isActive(id)? imguiRGBA(ACTIVE_CHECKED_CHECK_COLOR): imguiRGBA(CHECKED_CHECK_COLOR));
    else
      addGfxCmdRoundedRect(cx, cy, CHECK_SIZE, CHECK_SIZE, CHECK_SIZE/2-1/*DEFAULT_CHECK_CORNER_ROUNDING*/, imguiRGBA(DISABLED_CHECKED_CHECK_COLOR));
  }

  if (enabled)
    addGfxCmdText(x+BUTTON_HEIGHT, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, isHot(id) ? imguiRGBA(HOT_CHECK_TEXT_COLOR) : imguiRGBA(DEFAULT_CHECK_TEXT_COLOR));
  else
    addGfxCmdText(x+BUTTON_HEIGHT, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(DISABLED_CHECK_TEXT_COLOR));

  return res;
}

bool imguiCollapse(const char* text, bool checked/*, bool enabled*/)
{
  g_state.widgetId++;
  unsigned int id = (g_state.areaId<<16) | g_state.widgetId;
  
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  int w = g_state.widgetW;
  int h = BUTTON_HEIGHT;
  g_state.widgetY -= BUTTON_HEIGHT; // + DEFAULT_SPACING;

  const int cx = x+BUTTON_HEIGHT/2-CHECK_SIZE/2;
  const int cy = y+BUTTON_HEIGHT/2-CHECK_SIZE/2;

  bool over = inRect(x, y, w, h);
  bool res = buttonLogic(id, over);
  
  if (checked)
    addGfxCmdTriangle(cx, cy, CHECK_SIZE, CHECK_SIZE, 1, isActive(id)? imguiRGBA(ACTIVE_CHECKED_COLLAPSE_COLOR) : imguiRGBA(CHECKED_COLLAPSE_COLOR) );
  else
    addGfxCmdTriangle(cx, cy, CHECK_SIZE, CHECK_SIZE, 2, isActive(id)? imguiRGBA(ACTIVE_COLLAPSE_COLOR) : imguiRGBA(DEFAULT_COLLAPSE_COLOR));

  addGfxCmdText(x+BUTTON_HEIGHT, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, isHot(id) ? imguiRGBA(HOT_COLLAPSE_TEXT_COLOR) : imguiRGBA(DEFAULT_COLLAPSE_TEXT_COLOR));

  return res;
}

void imguiLabel(const char* text)
{
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  g_state.widgetY -= BUTTON_HEIGHT;
  addGfxCmdText(x, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(DEFAULT_LABEL_TEXT_COLOR));
}

void imguiLabelWithAlignedValue(const char* text, float value_, int size_)
{
  size_t txtlen = strlen(text);
  char valbuf[64];
  snprintf(valbuf, 64, "%.3f", value_);
  size_t vallen = strlen(valbuf);
  int nblanks = size_ - (vallen + txtlen);
  char tbuf[128];
  if (size_ > 128 || nblanks <= 0)
    imguiLabel("----------");
  else{
    strcpy(tbuf, text);
    memset(&tbuf[txtlen], ' ', nblanks);
    tbuf[txtlen+nblanks] = '\0';
    strcat(tbuf, valbuf);
    imguiLabel(tbuf);
  }
}

void imguiLabelWithColor(const char* text, unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  g_state.widgetY -= BUTTON_HEIGHT;
  addGfxCmdText(x, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(r,g,b,a));
}

void imguiValue(const char* text)
{
  const int x = g_state.widgetX;
  const int y = g_state.widgetY - BUTTON_HEIGHT;
  const int w = g_state.widgetW;
  g_state.widgetY -= BUTTON_HEIGHT;
  
  addGfxCmdText(x+w-BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_RIGHT, text, imguiRGBA(DEFAULT_VALUE_TEXT_COLOR));
}

void imguiValueWithColor(const char* text, unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  const int x = g_state.widgetX;
  const int y = g_state.widgetY - BUTTON_HEIGHT;
  const int w = g_state.widgetW;
  g_state.widgetY -= BUTTON_HEIGHT;
  
  addGfxCmdText(x+w-BUTTON_HEIGHT/2, y+BUTTON_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_RIGHT, text, imguiRGBA(r,g,b,a));
}

bool imguiSlider(const char* text, double* val, double vmin, double vmax, double vinc, bool enabled)
{
  g_state.widgetId++;
  unsigned int id = (g_state.areaId<<16) | g_state.widgetId;
  
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;
  int w = g_state.widgetW;
  int h = SLIDER_HEIGHT;
  g_state.widgetY -= SLIDER_HEIGHT + DEFAULT_SPACING;

  addGfxCmdRoundedRect(x, y, w, h, DEFAULT_SLIDER_CORNER_ROUNDING, imguiRGBA(DEFAULT_SLIDER_BACK_COLOR));

  const int range = w - SLIDER_MARKER_WIDTH;

  double u = (*val - vmin) / (vmax-vmin);
  if (u < 0) u = 0;
  if (u > 1) u = 1;
  int m = (int)(u * range);
  
  bool over = enabled && inRect(x+m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT);
  bool res = buttonLogic(id, over);
  bool valChanged = false;

  if (isActive(id))
  {
    if (g_state.wentActive)
    {
      g_state.dragX = g_state.mx;
      g_state.dragOrig = (float)u;
    }
    if (g_state.dragX != g_state.mx)
    {
      u = g_state.dragOrig + (double)(g_state.mx - g_state.dragX) / (double)range;
      if (u < 0) u = 0;
      if (u > 1) u = 1;
      *val = vmin + u*(vmax-vmin);
      *val = floor(*val / vinc)*vinc; // Snap to vinc
      m = (int)(u * range);
      valChanged = true;
    }
  }

  if (isActive(id))
    addGfxCmdRoundedRect(x+m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT, 4, imguiRGBA(ACTIVE_SLIDER_HANDLE_COLOR));
  else
    addGfxCmdRoundedRect(x+m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT, 4, isHot(id) ? imguiRGBA(HOT_SLIDER_HANDLE_COLOR) : imguiRGBA(DEFAULT_SLIDER_HANDLE_COLOR));

  // TODO: fix this, take a look at 'nicenum'.
  int digits = (int)(ceilf(log10f((float)vinc)));
  char fmt[16];
  snprintf(fmt, 16, "%%.%df", digits >= 0 ? 0 : -digits);
  char msg[128];
  snprintf(msg, 128, fmt, *val);
  
  if (enabled)
  {
    addGfxCmdText(x+SLIDER_HEIGHT/2, y+SLIDER_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, isHot(id) ? imguiRGBA(HOT_SLIDER_TEXT_COLOR) : imguiRGBA(DEFAULT_SLIDER_TEXT_COLOR));
    addGfxCmdText(x+w-SLIDER_HEIGHT/2, y+SLIDER_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_RIGHT, msg, isHot(id) ? imguiRGBA(HOT_SLIDER_TEXT_COLOR) : imguiRGBA(DEFAULT_SLIDER_TEXT_COLOR));
  }else{
    addGfxCmdText(x+SLIDER_HEIGHT/2, y+SLIDER_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_LEFT, text, imguiRGBA(DISABLED_SLIDER_TEXT_COLOR));
    addGfxCmdText(x+w-SLIDER_HEIGHT/2, y+SLIDER_HEIGHT/2-TEXT_HEIGHT/2, IMGUI_ALIGN_RIGHT, msg, imguiRGBA(DISABLED_SLIDER_TEXT_COLOR));
  }

  return res || valChanged;
}

bool imguiSlider(const char* text, int* val, int vmin, int vmax, int vinc, bool enabled)
{
	double val_d = static_cast<double>(*val);
	bool res = imguiSlider(text, &val_d, static_cast<double>(vmin), static_cast<double>(vmax), static_cast<double>(vinc), enabled);
	*val = static_cast<int>(val_d);
	return res;
}

void imguiProgressBar(const float progress, const float t)
{
  int x = g_state.widgetX;
  int y = g_state.widgetY - BUTTON_HEIGHT;

  g_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

  float p = clamp(progress, 0.f, 1.f);

  addGfxCmdRoundedRect(x, y, PROGRESS_WIDTH, BUTTON_HEIGHT, BUTTON_HEIGHT/2-1, imguiRGBA(DEFAULT_PROGRESSBAR_BACK_COLOR));
  addGfxCmdRoundedRect(x, y, std::max(20.f, p*PROGRESS_WIDTH) , BUTTON_HEIGHT, BUTTON_HEIGHT/2-1, imguiRGBA(0, 192, 255, 128 + (unsigned char)(127*t)));
}

void imguiIndent()
{
  g_state.widgetX += INTEND_SIZE;
  g_state.widgetW -= INTEND_SIZE;
}

void imguiUnindent()
{
  g_state.widgetX -= INTEND_SIZE;
  g_state.widgetW += INTEND_SIZE;
}

void imguiSeparator()
{
  g_state.widgetY -= DEFAULT_SPACING*3;
}

void imguiDrawText(int x, int y, int align, const char* text, unsigned int color)
{
  addGfxCmdText(x, y, align, text, color);
}

// cmake:sourcegroup=Gui
