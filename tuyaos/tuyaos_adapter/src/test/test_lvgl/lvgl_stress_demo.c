#include <stdint.h>

#include "lvgl.h"
#include "lvgl_stress_demo.h"
#include "tkl_output.h"

#if (CONFIG_LVGL && CONFIG_LVGL_V9)

#define TUYA_LVGL_STRESS_TIME_STEP_MS   50U
#define TUYA_LVGL_STRESS_LIST_ITEMS     6U
#define TUYA_LVGL_STRESS_STATUS_HEIGHT  48

static lv_timer_t *s_stress_timer;
static lv_obj_t *s_root_page;
static lv_obj_t *s_dynamic_layer;
static lv_obj_t *s_status_label;
static lv_obj_t *s_main_page;
static lv_obj_t *s_textarea;
static uint32_t s_mem_free_start;
static uint32_t s_cycle_count;
static int16_t s_state = -1;

static void set_y_anim(void *obj, int32_t v)
{
    lv_obj_set_y((lv_obj_t *)obj, v);
}

static void set_width_anim(void *obj, int32_t v)
{
    lv_obj_set_width((lv_obj_t *)obj, v);
}

static void arc_set_value_anim(void *obj, int32_t v)
{
    lv_arc_set_value((lv_obj_t *)obj, v);
}

static void stress_mem_check(void)
{
    lv_result_t res = lv_mem_test();
    lv_mem_monitor_t mon;

    if (res != LV_RESULT_OK) {
        tkl_log_output("lvgl stress memory integrity error\r\n");
        return;
    }

    lv_mem_monitor(&mon);
    if (s_mem_free_start == 0U) {
        s_mem_free_start = mon.free_size;
    }

    tkl_log_output("lvgl stress mem leak=%u, frag=%u%%\r\n",
                   (unsigned int)((s_mem_free_start > mon.free_size) ? (s_mem_free_start - mon.free_size) : 0U),
                   (unsigned int)mon.frag_pct);
}

static void stress_delete_later(lv_obj_t *obj, uint32_t delay_ms)
{
    if (obj != NULL) {
        lv_obj_delete_delayed(obj, delay_ms);
    }
}

static lv_obj_t *stress_dynamic_parent_get(void)
{
    return (s_dynamic_layer != NULL) ? s_dynamic_layer : lv_screen_active();
}

static void stress_status_update(void)
{
    if (s_status_label != NULL) {
        lv_label_set_text_fmt(s_status_label,
                              "LVGL stress  cycle=%u  state=%d",
                              (unsigned int)s_cycle_count,
                              (int)s_state);
    }
}

static void stress_screen_create(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *status_bar;

    lv_anim_delete_all();
    lv_obj_clean(screen);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xF4F6F8), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    s_root_page = lv_obj_create(screen);
    lv_obj_remove_style_all(s_root_page);
    lv_obj_set_size(s_root_page, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root_page, lv_color_hex(0xF4F6F8), 0);
    lv_obj_set_style_bg_opa(s_root_page, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_root_page, LV_OBJ_FLAG_SCROLLABLE);

    status_bar = lv_obj_create(s_root_page);
    lv_obj_remove_style_all(status_bar);
    lv_obj_set_size(status_bar, LV_PCT(100), TUYA_LVGL_STRESS_STATUS_HEIGHT);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0xDDE3EA), 0);
    lv_obj_set_style_bg_opa(status_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(status_bar, 16, 0);
    lv_obj_set_style_pad_right(status_bar, 16, 0);
    lv_obj_set_flex_flow(status_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(status_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_status_label = lv_label_create(status_bar);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x20242A), 0);

    s_dynamic_layer = lv_obj_create(s_root_page);
    lv_obj_remove_style_all(s_dynamic_layer);
    lv_obj_set_size(s_dynamic_layer, LV_PCT(100), LV_VER_RES - TUYA_LVGL_STRESS_STATUS_HEIGHT);
    lv_obj_align(s_dynamic_layer, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(s_dynamic_layer, lv_color_hex(0xF4F6F8), 0);
    lv_obj_set_style_bg_opa(s_dynamic_layer, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_dynamic_layer, LV_OBJ_FLAG_SCROLLABLE);

    stress_status_update();
}

static void stress_create_base_page(void)
{
    lv_obj_t *obj;

    s_main_page = lv_obj_create(stress_dynamic_parent_get());
    lv_obj_set_size(s_main_page, LV_PCT(48), LV_PCT(96));
    lv_obj_align(s_main_page, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_set_flex_flow(s_main_page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(s_main_page, 8, 0);
    lv_obj_set_style_pad_gap(s_main_page, 8, 0);

    obj = lv_button_create(s_main_page);
    lv_obj_set_size(obj, 140, 64);
    obj = lv_label_create(obj);
    lv_label_set_text(obj, "Stress\n" LV_SYMBOL_OK " " LV_SYMBOL_CLOSE " " LV_SYMBOL_WIFI);
    lv_obj_center(obj);
}

static void stress_create_tabview(void)
{
#if LV_USE_TABVIEW
    lv_obj_t *tabview = lv_tabview_create(stress_dynamic_parent_get());
    lv_obj_set_size(tabview, LV_PCT(48), LV_PCT(45));
    lv_obj_align(tabview, LV_ALIGN_BOTTOM_RIGHT, -4, -4);
    lv_tabview_set_tab_bar_position(tabview, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tabview, 48);

    lv_obj_t *tab = lv_tabview_add_tab(tabview, "First");
    lv_label_create(tab);
    lv_label_set_text(lv_obj_get_child(tab, 0), "LVGL stress tab");

    tab = lv_tabview_add_tab(tabview, "Second");
    lv_obj_t *slider = lv_slider_create(tab);
    lv_obj_set_width(slider, LV_PCT(80));
    lv_slider_set_value(slider, 70, LV_ANIM_ON);

    tab = lv_tabview_add_tab(tabview, LV_SYMBOL_EDIT " Edit");
    lv_tabview_set_active(tabview, 1, LV_ANIM_ON);
    stress_delete_later(tabview, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif
}

static void stress_create_animated_button(void)
{
    if (s_main_page == NULL) {
        return;
    }

    lv_obj_t *btn = lv_button_create(s_main_page);
    lv_obj_set_size(btn, 120, 56);
    lv_obj_add_state(btn, LV_STATE_DISABLED);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text_fmt(label, "Count\n%d", (int)s_state);
    lv_obj_center(label);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, btn);
    lv_anim_set_time(&a, TUYA_LVGL_STRESS_TIME_STEP_MS * 4U);
    lv_anim_set_exec_cb(&a, set_width_anim);
    lv_anim_set_values(&a, 100, 220);
    lv_anim_set_playback_time(&a, TUYA_LVGL_STRESS_TIME_STEP_MS * 4U);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    stress_delete_later(btn, TUYA_LVGL_STRESS_TIME_STEP_MS * 12U);
}

static void stress_create_textarea(void)
{
#if LV_USE_TEXTAREA
    s_textarea = lv_textarea_create(stress_dynamic_parent_get());
    lv_obj_set_size(s_textarea, LV_PCT(42), LV_PCT(22));
    lv_obj_align(s_textarea, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_textarea_set_placeholder_text(s_textarea, "Stress input");
    lv_textarea_set_text(s_textarea, "LVGL v9 textarea");
#endif
}

static void stress_create_value_widgets(void)
{
    if (s_main_page == NULL) {
        return;
    }

#if LV_USE_BAR
    lv_obj_t *bar = lv_bar_create(s_main_page);
    lv_obj_set_width(bar, LV_PCT(90));
    lv_bar_set_range(bar, -1000, 2000);
    lv_bar_set_value(bar, 1800, LV_ANIM_ON);
    lv_bar_set_start_value(bar, -500, LV_ANIM_ON);
    stress_delete_later(bar, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif

#if LV_USE_SLIDER
    lv_obj_t *slider = lv_slider_create(s_main_page);
    lv_obj_set_width(slider, LV_PCT(90));
    lv_slider_set_value(slider, 85, LV_ANIM_ON);
    stress_delete_later(slider, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U + 25U);
#endif

#if LV_USE_SWITCH
    lv_obj_t *sw = lv_switch_create(s_main_page);
    lv_obj_add_state(sw, LV_STATE_CHECKED);
    stress_delete_later(sw, TUYA_LVGL_STRESS_TIME_STEP_MS * 6U);
#endif
}

static void stress_create_window_calendar(void)
{
#if LV_USE_WIN
    lv_obj_t *win = lv_win_create(stress_dynamic_parent_get());
    lv_obj_set_size(win, LV_PCT(48), LV_PCT(45));
    lv_obj_align(win, LV_ALIGN_BOTTOM_RIGHT, -4, -4);
    lv_win_add_title(win, "Window");
    lv_win_add_button(win, LV_SYMBOL_CLOSE, 42);
    lv_win_add_button(win, LV_SYMBOL_DOWN, 42);

#if LV_USE_CALENDAR
    lv_obj_t *calendar = lv_calendar_create(lv_win_get_content(win));
    lv_obj_set_size(calendar, LV_PCT(95), LV_PCT(90));
    lv_calendar_set_today_date(calendar, 2026, 5, 12);
    lv_calendar_set_month_shown(calendar, 2026, 5);
#endif

    stress_delete_later(win, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif
}

static void stress_create_keyboard(void)
{
#if LV_USE_KEYBOARD && LV_USE_TEXTAREA
    lv_obj_t *kb = lv_keyboard_create(stress_dynamic_parent_get());
    lv_obj_set_size(kb, LV_PCT(100), LV_PCT(35));
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_TEXT_UPPER);
    if (s_textarea != NULL) {
        lv_keyboard_set_textarea(kb, s_textarea);
    }

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, kb);
    lv_anim_set_values(&a, LV_VER_RES, LV_VER_RES - lv_obj_get_height(kb));
    lv_anim_set_time(&a, TUYA_LVGL_STRESS_TIME_STEP_MS * 3U);
    lv_anim_set_exec_cb(&a, set_y_anim);
    lv_anim_start(&a);

    stress_delete_later(kb, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif
}

static void stress_create_selectors(void)
{
    if (s_main_page == NULL) {
        return;
    }

#if LV_USE_DROPDOWN
    lv_obj_t *dropdown = lv_dropdown_create(s_main_page);
    lv_dropdown_set_options(dropdown, "Zero\nOne\nTwo\nThree\nFour\nFive");
    lv_dropdown_set_selected(dropdown, 2);
    lv_dropdown_open(dropdown);
    stress_delete_later(dropdown, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif

#if LV_USE_ROLLER
    lv_obj_t *roller = lv_roller_create(s_main_page);
    lv_roller_set_options(roller, "Zero\nOne\nTwo\nThree\nFour\nFive", LV_ROLLER_MODE_INFINITE);
    lv_roller_set_selected(roller, 3, LV_ANIM_ON);
    stress_delete_later(roller, TUYA_LVGL_STRESS_TIME_STEP_MS * 10U);
#endif
}

static void stress_create_arc_chart(void)
{
    if (s_main_page == NULL) {
        return;
    }

#if LV_USE_ARC
    lv_obj_t *arc = lv_arc_create(s_main_page);
    lv_obj_set_size(arc, 120, 120);
    lv_arc_set_range(arc, 0, 100);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arc);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, TUYA_LVGL_STRESS_TIME_STEP_MS * 10U);
    lv_anim_set_playback_time(&a, TUYA_LVGL_STRESS_TIME_STEP_MS * 6U);
    lv_anim_set_repeat_count(&a, 2);
    lv_anim_set_exec_cb(&a, arc_set_value_anim);
    lv_anim_start(&a);
    stress_delete_later(arc, TUYA_LVGL_STRESS_TIME_STEP_MS * 18U);
#endif

#if LV_USE_CHART
    lv_obj_t *chart = lv_chart_create(s_main_page);
    lv_obj_set_size(chart, LV_PCT(90), 120);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, 8);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, -40, 120);
    lv_chart_series_t *ser = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    for (int i = 0; i < 8; i++) {
        lv_chart_set_next_value(chart, ser, (i * 23) % 120 - 30);
    }
    stress_delete_later(chart, TUYA_LVGL_STRESS_TIME_STEP_MS * 10U);
#endif
}

static void stress_create_msgbox_tileview(void)
{
#if LV_USE_MSGBOX
    lv_obj_t *mbox = lv_msgbox_create(stress_dynamic_parent_get());
    lv_msgbox_add_title(mbox, "Stress");
    lv_msgbox_add_text(mbox, "Creating and deleting LVGL objects.");
    lv_msgbox_add_footer_button(mbox, "Ok");
    lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_align(mbox, LV_ALIGN_RIGHT_MID, -12, 0);
    stress_delete_later(mbox, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif

#if LV_USE_TILEVIEW
    lv_obj_t *tileview = lv_tileview_create(stress_dynamic_parent_get());
    lv_obj_set_size(tileview, 220, 220);
    lv_obj_align(tileview, LV_ALIGN_BOTTOM_RIGHT, -8, -8);

    lv_obj_t *tile = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_ALL);
    lv_obj_t *label = lv_label_create(tile);
    lv_label_set_text(label, "Tile 0,0");
    lv_obj_center(label);

    tile = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_ALL);
    label = lv_label_create(tile);
    lv_label_set_text(label, "Tile 1,0");
    lv_obj_center(label);

    tile = lv_tileview_add_tile(tileview, 1, 1, LV_DIR_ALL);
    label = lv_label_create(tile);
    lv_label_set_text(label, "Tile 1,1");
    lv_obj_center(label);

    lv_tileview_set_tile_by_index(tileview, 1, 1, LV_ANIM_ON);
    stress_delete_later(tileview, TUYA_LVGL_STRESS_TIME_STEP_MS * 10U);
#endif
}

static void stress_create_list_table(void)
{
    if (s_main_page == NULL) {
        return;
    }

#if LV_USE_LIST
    lv_obj_t *list = lv_list_create(s_main_page);
    lv_obj_set_size(list, LV_PCT(92), 130);
    lv_list_add_text(list, "List");
    for (uint32_t i = 0; i < TUYA_LVGL_STRESS_LIST_ITEMS; i++) {
        lv_obj_t *btn = lv_list_add_button(list, LV_SYMBOL_OK, "Scrolling item");
        if ((i % 2U) == 0U) {
            stress_delete_later(btn, TUYA_LVGL_STRESS_TIME_STEP_MS * (i + 2U));
        }
    }
    stress_delete_later(list, TUYA_LVGL_STRESS_TIME_STEP_MS * 10U);
#endif

#if LV_USE_TABLE
    lv_obj_t *table = lv_table_create(s_main_page);
    lv_obj_set_size(table, LV_PCT(92), 120);
    lv_table_set_cell_value(table, 0, 0, "0,0");
    lv_table_set_cell_value_fmt(table, 1, 1, "%d,%d", 1, 1);
    lv_table_set_cell_value_fmt(table, 2, 0, "state %d", (int)s_state);
    stress_delete_later(table, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif
}

static void stress_create_inputs(void)
{
    if (s_main_page == NULL) {
        return;
    }

#if LV_USE_SPINBOX
    lv_obj_t *spinbox = lv_spinbox_create(s_main_page);
    lv_spinbox_set_digit_format(spinbox, 6, 3);
    lv_spinbox_set_value(spinbox, 5678);
    lv_spinbox_set_step(spinbox, 10);
    lv_spinbox_increment(spinbox);
    lv_spinbox_set_step(spinbox, 100);
    lv_spinbox_increment(spinbox);
    stress_delete_later(spinbox, TUYA_LVGL_STRESS_TIME_STEP_MS * 8U);
#endif

#if LV_USE_CHECKBOX
    lv_obj_t *checkbox = lv_checkbox_create(s_main_page);
    lv_checkbox_set_text(checkbox, "Option");
    lv_obj_add_state(checkbox, LV_STATE_CHECKED);
    stress_delete_later(checkbox, TUYA_LVGL_STRESS_TIME_STEP_MS * 6U);
#endif

#if LV_USE_TEXTAREA
    if (s_textarea != NULL) {
        lv_textarea_set_cursor_pos(s_textarea, 0);
        lv_textarea_add_char(s_textarea, '0' + (s_state % 10));
        lv_textarea_add_text(s_textarea, " " LV_SYMBOL_EDIT);
        lv_textarea_set_one_line(s_textarea, (s_state & 1) ? true : false);
    }
#endif

    lv_obj_scroll_by(s_main_page, 0, 20, LV_ANIM_ON);
}

static void stress_cleanup_cycle(void)
{
    lv_anim_delete_all();
    if (s_dynamic_layer != NULL) {
        lv_obj_clean(s_dynamic_layer);
        lv_obj_invalidate(s_dynamic_layer);
    }
    s_main_page = NULL;
    s_textarea = NULL;
}

static void stress_screen_destroy(void)
{
    lv_anim_delete_all();
    lv_obj_clean(lv_screen_active());
    s_root_page = NULL;
    s_dynamic_layer = NULL;
    s_status_label = NULL;
    s_main_page = NULL;
    s_textarea = NULL;
}

static void stress_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    switch (s_state) {
        case -1:
            stress_mem_check();
            break;
        case 0:
            stress_create_base_page();
            break;
        case 1:
            stress_create_tabview();
            break;
        case 2:
            stress_create_animated_button();
            break;
        case 3:
            stress_create_textarea();
            break;
        case 5:
            if (s_textarea != NULL) {
                lv_textarea_set_one_line(s_textarea, true);
            }
            break;
        case 6:
            if (s_main_page != NULL) {
                lv_obj_set_flex_flow(s_main_page, LV_FLEX_FLOW_COLUMN_WRAP);
            }
            break;
        case 7:
            stress_create_value_widgets();
            break;
        case 8:
            stress_create_window_calendar();
            break;
        case 9:
            if (s_textarea != NULL) {
                lv_textarea_set_text(s_textarea, "A long text used to stress textarea scrolling and redraw.");
            }
            break;
        case 10:
            stress_create_keyboard();
            break;
        case 11:
            stress_create_selectors();
            break;
        case 13:
            stress_create_arc_chart();
            break;
        case 14:
            stress_create_msgbox_tileview();
            break;
        case 18:
            stress_create_list_table();
            break;
        case 20:
            if (s_main_page != NULL) {
                lv_obj_set_flex_flow(s_main_page, LV_FLEX_FLOW_ROW_WRAP);
            }
            break;
        case 22:
            stress_create_inputs();
            break;
        case 29:
            lv_anim_delete_all();
            if (s_main_page != NULL) {
                lv_obj_clean(s_main_page);
            }
            if (s_textarea != NULL) {
                lv_obj_delete(s_textarea);
                s_textarea = NULL;
            }
            break;
        case 31:
            stress_cleanup_cycle();
            s_cycle_count++;
            s_state = -2;
            break;
        default:
            break;
    }

    s_state++;
    stress_status_update();
}

void tuya_lvgl_stress_create(void)
{
    if (s_stress_timer != NULL) {
        return;
    }

    stress_screen_create();
    s_mem_free_start = 0;
    s_cycle_count = 0;
    s_state = -1;
    stress_status_update();
    tkl_log_output("lvgl stress start\r\n");
    s_stress_timer = lv_timer_create(stress_timer_cb, TUYA_LVGL_STRESS_TIME_STEP_MS, NULL);
}

void tuya_lvgl_stress_pause(void)
{
    if (s_stress_timer != NULL) {
        lv_timer_pause(s_stress_timer);
    }
}

void tuya_lvgl_stress_resume(void)
{
    if (s_stress_timer != NULL) {
        lv_timer_resume(s_stress_timer);
    }
}

void tuya_lvgl_stress_destroy(void)
{
    if (s_stress_timer != NULL) {
        lv_timer_delete(s_stress_timer);
        s_stress_timer = NULL;
    }

    stress_screen_destroy();
    s_mem_free_start = 0;
    s_cycle_count = 0;
    s_state = -1;
}

#endif
