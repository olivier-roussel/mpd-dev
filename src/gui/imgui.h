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

#ifndef IMGUI_H
#define IMGUI_H

#include <Eigen/Core>

static const int DEFAULT_AREA_CORNER_ROUNDING = 3;
static const int DEFAULT_BUTTON_CORNER_ROUNDING = 3;
static const int DEFAULT_SLIDER_CORNER_ROUNDING = 2;
static const int DEFAULT_CHECK_CORNER_ROUNDING = 5;

static const Eigen::Vector4i DISABLED_SLIDER_TEXT_COLOR = Eigen::Vector4i(128, 128, 128, 200);
static const Eigen::Vector4i DEFAULT_SLIDER_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i HOT_SLIDER_TEXT_COLOR = Eigen::Vector4i(0, 196, 255, 192);
static const Eigen::Vector4i HOT_SLIDER_HANDLE_COLOR = Eigen::Vector4i(0, 196, 255, 128);
static const Eigen::Vector4i ACTIVE_SLIDER_HANDLE_COLOR= Eigen::Vector4i(0, 196, 255, 255);
static const Eigen::Vector4i DEFAULT_SLIDER_HANDLE_COLOR = Eigen::Vector4i(255, 255, 255, 64);
static const Eigen::Vector4i DEFAULT_SLIDER_BACK_COLOR = Eigen::Vector4i(0, 0, 0, 128);

static const Eigen::Vector4i DEFAULT_SCROLLAREA_BACK_COLOR = Eigen::Vector4i(0, 0, 0, 192);
static const Eigen::Vector4i DEFAULT_SCROLLAREA_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 128);
static const Eigen::Vector4i DEFAULT_SCROLLAREA_BG_COLOR = Eigen::Vector4i(0, 0, 0, 196);
static const Eigen::Vector4i ACTIVE_SCROLLAREA_BAR_COLOR = Eigen::Vector4i(0, 196, 255, 196);
static const Eigen::Vector4i HOT_SCROLLAREA_BAR_COLOR = Eigen::Vector4i(0, 196, 255, 96);
static const Eigen::Vector4i DEFAULT_SCROLLAREA_BAR_COLOR = Eigen::Vector4i(255, 255, 255, 64);

static const Eigen::Vector4i DEFAULT_SIMPLEAREA_BACK_COLOR = Eigen::Vector4i(0, 0, 0, 192);
static const Eigen::Vector4i DEFAULT_SIMPLEAREA_BG_COLOR = Eigen::Vector4i(0, 0, 0, 196);
static const Eigen::Vector4i DEFAULT_SIMPLEAREA_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 128);

static const Eigen::Vector4i DEFAULT_BUTTON_COLOR = Eigen::Vector4i(128, 128, 128, 96);
static const Eigen::Vector4i ACTIVE_BUTTON_COLOR = Eigen::Vector4i(128, 128, 128, 196);
static const Eigen::Vector4i DEFAULT_BUTTON_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i DISABLED_BUTTON_TEXT_COLOR = Eigen::Vector4i(128, 128, 128, 200);
static const Eigen::Vector4i HOT_BUTTON_TEXT_COLOR = Eigen::Vector4i(0, 196, 255, 255);

static const Eigen::Vector4i SELECTED_ITEM_COLOR = Eigen::Vector4i(0, 196, 255, 96);
static const Eigen::Vector4i ACTIVE_ITEM_COLOR = Eigen::Vector4i(0, 196, 255, 196);
static const Eigen::Vector4i DEFAULT_ITEM_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i DISABLED_ITEM_TEXT_COLOR = Eigen::Vector4i(128, 128, 128, 200);

static const Eigen::Vector4i DEFAULT_CHECK_COLOR = Eigen::Vector4i(128, 128, 128, 96);
static const Eigen::Vector4i ACTIVE_CHECK_COLOR = Eigen::Vector4i(128, 128, 128, 196);
static const Eigen::Vector4i CHECKED_CHECK_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i ACTIVE_CHECKED_CHECK_COLOR = Eigen::Vector4i(255, 255, 255, 255);
static const Eigen::Vector4i DISABLED_CHECKED_CHECK_COLOR = Eigen::Vector4i(128, 128, 128, 200);
static const Eigen::Vector4i DEFAULT_CHECK_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i HOT_CHECK_TEXT_COLOR = Eigen::Vector4i(0, 196, 255, 255);
static const Eigen::Vector4i DISABLED_CHECK_TEXT_COLOR = Eigen::Vector4i(128, 128, 128, 200);

static const Eigen::Vector4i DEFAULT_COLLAPSE_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i ACTIVE_COLLAPSE_COLOR = Eigen::Vector4i(255, 255, 255, 255);
static const Eigen::Vector4i CHECKED_COLLAPSE_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i ACTIVE_CHECKED_COLLAPSE_COLOR = Eigen::Vector4i(255, 255, 255, 255);
static const Eigen::Vector4i DEFAULT_COLLAPSE_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 200);
static const Eigen::Vector4i HOT_COLLAPSE_TEXT_COLOR = Eigen::Vector4i(0, 196, 255, 255);

static const Eigen::Vector4i DEFAULT_LABEL_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 255);

static const Eigen::Vector4i DEFAULT_VALUE_TEXT_COLOR = Eigen::Vector4i(255, 255, 255, 200);

static const Eigen::Vector4i DEFAULT_PROGRESSBAR_BACK_COLOR = Eigen::Vector4i(64, 64, 64, 64);

enum imguiMouseButton
{
	IMGUI_MBUT_LEFT = 0x01, 
	IMGUI_MBUT_RIGHT = 0x02 
};

enum imguiTextAlign
{
	IMGUI_ALIGN_LEFT,
	IMGUI_ALIGN_CENTER,
	IMGUI_ALIGN_RIGHT
};

inline unsigned int imguiRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a=255)
{
	return (r) | (g << 8) | (b << 16) | (a << 24);
}

inline unsigned int imguiRGBA(const Eigen::Vector4i& col_)
{
	return (col_[0]) | (col_[1] << 8) | (col_[2] << 16) | (col_[3] << 24);
}

void imguiBeginFrame(int mx, int my, unsigned char mbut, int scroll);
void imguiEndFrame();

bool imguiBeginScrollArea(const char* name, int x, int y, int w, int h, int* scroll);
void imguiEndScrollArea();

void imguiBeginSimpleArea(const char* name, int x, int y, int w, int h);
void imguiEndSimpleArea();

void imguiIndent();
void imguiUnindent();
void imguiSeparator();

bool imguiButton(const char* text, bool enabled = true);
bool imguiItem(const char* text, bool selected = false, bool enabled = true);
bool imguiCheck(const char* text, bool checked, bool enabled = true);
bool imguiCollapse(const char* text, bool checked, bool enabled = true);
void imguiLabel(const char* text);
void imguiLabelWithColor(const char* text, unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
void imguiLabelWithAlignedValue(const char* text, float value_, int size_);
void imguiValue(const char* text);
void imguiValueWithColor(const char* text, unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
bool imguiSlider(const char* text, double* val, double vmin, double vmax, double vinc, bool enabled = true);

void imguiDrawText(int x, int y, int align, const char* text, unsigned int color);

void imguiProgressBar(float progress, const float t=1.f);

// Pull render interface.
enum imguiGfxCmdType
{
	IMGUI_GFXCMD_RECT,
	IMGUI_GFXCMD_TRIANGLE,
	IMGUI_GFXCMD_TEXT,
	IMGUI_GFXCMD_SCISSOR
};

struct imguiGfxRect
{
	short x,y,w,h,r;
};

struct imguiGfxText
{
	short x,y,align;
	const char* text;
};

struct imguiGfxCmd
{
	char type;
	char flags;
	char pad[2];
	unsigned int col;
	union
	{
		imguiGfxRect rect;
		imguiGfxText text;
	};
};

const imguiGfxCmd* imguiGetRenderQueue();
int imguiGetRenderQueueSize();


#endif // IMGUI_H

// cmake:sourcegroup=Gui
