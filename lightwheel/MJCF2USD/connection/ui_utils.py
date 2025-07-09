# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

#   str_builder

import asyncio
import os
import subprocess
import sys
from cmath import inf

import carb.settings
import omni.appwindow
import omni.ext
import omni.ui as ui
from omni.kit.window.extensions import SimpleCheckBox
from omni.kit.window.filepicker import FilePickerDialog
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH

# from .callbacks import on_copy_to_clipboard, on_docs_link_clicked, on_open_folder_clicked, on_open_IDE_clicked
from .style import (
    BUTTON_WIDTH,
    COLOR_W,
    COLOR_X,
    COLOR_Y,
    COLOR_Z,
    get_option_style,
    get_style,
)


def add_line_rect_flourish(draw_line=True):
    """Aesthetic element that adds a Line + Rectangle after all UI elements in the row.

    Args:
        draw_line (bool, optional): Set false to only draw rectangle. Defaults to True.
    """
    if draw_line:
        ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), alignment=ui.Alignment.CENTER)
    ui.Spacer(width=10)
    with ui.Frame(width=0):
        with ui.VStack():
            with ui.Placer(offset_x=0, offset_y=7):
                ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
    ui.Spacer(width=5)


def format_tt(tt):
    import string

    formated = ""
    i = 0
    for w in tt.split():
        if w.isupper():
            formated += w + " "
        elif len(w) > 3 or i == 0:
            formated += string.capwords(w) + " "
        else:
            formated += w.lower() + " "
        i += 1
    return formated


def add_folder_picker_icon(
    on_click_fn,
    item_filter_fn=None,
    bookmark_label=None,
    bookmark_path=None,
    dialog_title="Select Output Folder",
    button_title="Select Folder",
    size=24,
):
    def open_file_picker():
        def on_selected(filename, path):
            on_click_fn(filename, path)
            file_picker.hide()

        def on_canceled(a, b):
            file_picker.hide()

        file_picker = FilePickerDialog(
            dialog_title,
            allow_multi_selection=False,
            apply_button_label=button_title,
            click_apply_handler=lambda a, b: on_selected(a, b),
            click_cancel_handler=lambda a, b: on_canceled(a, b),
            item_filter_fn=item_filter_fn,
            enable_versioning_pane=True,
        )
        if bookmark_label and bookmark_path:
            file_picker.toggle_bookmark_from_path(bookmark_label, bookmark_path, True)

    with ui.Frame(width=0, tooltip=button_title):
        ui.Button(
            name="IconButton",
            width=size,
            height=size,
            clicked_fn=open_file_picker,
            style=get_style()["IconButton.Image::FolderPicker"],
            alignment=ui.Alignment.RIGHT_CENTER,
        )
    return open_file_picker


def btn_builder(label="", type="button", text="button", tooltip="", on_clicked_fn=None):
    """Creates a stylized button.

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "button".
        text (str, optional): Text rendered on the button. Defaults to "button".
        tooltip (str, optional): Tooltip to display over the Label. Defaults to "".
        on_clicked_fn (Callable, optional): Call-back function when clicked. Defaults to None.

    Returns:
        ui.Button: Button
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        btn = ui.Button(
            text.upper(),
            name="Button",
            width=BUTTON_WIDTH,
            clicked_fn=on_clicked_fn,
            style=get_style(),
            alignment=ui.Alignment.LEFT_CENTER,
        )
        ui.Spacer(width=5)
        add_line_rect_flourish(True)
        # ui.Spacer(width=ui.Fraction(1))
        # ui.Spacer(width=10)
        # with ui.Frame(width=0):
        #     with ui.VStack():
        #         with ui.Placer(offset_x=0, offset_y=7):
        #             ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
        # ui.Spacer(width=5)
    return btn


def cb_builder(label="", type="checkbox", default_val=False, tooltip="", on_clicked_fn=None):
    """Creates a Stylized Checkbox

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "checkbox".
        default_val (bool, optional): Checked is True, Unchecked is False. Defaults to False.
        tooltip (str, optional): Tooltip to display over the Label. Defaults to "".
        on_clicked_fn (Callable, optional): Call-back function when clicked. Defaults to None.

    Returns:
        ui.SimpleBoolModel: model
    """

    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        model = ui.SimpleBoolModel()
        callable = on_clicked_fn
        if callable is None:
            callable = lambda x: None
        SimpleCheckBox(default_val, callable, model=model)

        add_line_rect_flourish()
        return model


def dropdown_builder(
    label="", type="dropdown", default_val=0, items=["Option 1", "Option 2", "Option 3"], tooltip="", on_clicked_fn=None
):
    """Creates a Stylized Dropdown Combobox

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "dropdown".
        default_val (int, optional): Default index of dropdown items. Defaults to 0.
        items (list, optional): List of items for dropdown box. Defaults to ["Option 1", "Option 2", "Option 3"].
        tooltip (str, optional): Tooltip to display over the Label. Defaults to "".
        on_clicked_fn (Callable, optional): Call-back function when clicked. Defaults to None.

    Returns:
        AbstractItemModel: model
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        combo_box = ui.ComboBox(
            default_val, *items, name="ComboBox", width=ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER
        ).model
        add_line_rect_flourish(False)

        def on_clicked_wrapper(model, val):
            on_clicked_fn(items[model.get_item_value_model().as_int])

        if on_clicked_fn is not None:
            combo_box.add_item_changed_fn(on_clicked_wrapper)

    return combo_box


def float_builder(label="", type="floatfield", default_val=0, tooltip="", min=-inf, max=inf, step=0.1, format="%.2f"):
    """Creates a Stylized Floatfield Widget

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "floatfield".
        default_val (int, optional): Default Value of UI element. Defaults to 0.
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".

    Returns:
        AbstractValueModel: model
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        float_field = ui.FloatDrag(
            name="FloatField",
            width=ui.Fraction(1),
            height=0,
            alignment=ui.Alignment.LEFT_CENTER,
            min=min,
            max=max,
            step=step,
            format=format,
        ).model
        float_field.set_value(default_val)
        add_line_rect_flourish(False)
        return float_field


def str_builder(
    label="",
    type="stringfield",
    default_val=" ",
    tooltip="",
    on_clicked_fn=None,
    use_folder_picker=False,
    read_only=False,
    item_filter_fn=None,
    bookmark_label=None,
    bookmark_path=None,
    folder_dialog_title="Select Output Folder",
    folder_button_title="Select Folder",
):
    """Creates a Stylized Stringfield Widget

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "stringfield".
        default_val (str, optional): Text to initialize in Stringfield. Defaults to " ".
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".
        use_folder_picker (bool, optional): Add a folder picker button to the right. Defaults to False.
        read_only (bool, optional): Prevents editing. Defaults to False.
        item_filter_fn (Callable, optional): filter function to pass to the FilePicker
        bookmark_label (str, optional): bookmark label to pass to the FilePicker
        bookmark_path (str, optional): bookmark path to pass to the FilePicker
    Returns:
        AbstractValueModel: model of Stringfield
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        str_field = ui.StringField(
            name="StringField", width=ui.Fraction(1), height=0, alignment=ui.Alignment.LEFT_CENTER, read_only=read_only
        ).model
        str_field.set_value(default_val)

        if use_folder_picker:

            def update_field(filename, path):
                if filename == "":
                    val = path
                elif filename[0] != "/" and path[-1] != "/":
                    val = path + "/" + filename
                elif filename[0] == "/" and path[-1] == "/":
                    val = path + filename[1:]
                else:
                    val = path + filename
                str_field.set_value(val)

            add_folder_picker_icon(
                update_field,
                item_filter_fn,
                bookmark_label,
                bookmark_path,
                dialog_title=folder_dialog_title,
                button_title=folder_button_title,
            )
        else:
            add_line_rect_flourish(False)
        return str_field
