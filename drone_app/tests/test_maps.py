"""Map selection through Matplotlib's mouse events, without a GUI or drone."""

from pathlib import Path
import sys
from types import SimpleNamespace

from matplotlib.backend_bases import MouseEvent
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from drone_app.maps import TagPlot


@pytest.fixture
def plot():
    figure = Figure(figsize=(11, 5), layout="constrained")
    FigureCanvasAgg(figure)
    result = TagPlot(figure, show=False, process_events=False)
    result.markers[5], = result.position_ax.plot([], [], "o")
    result.map.tags[5] = (2.5, -1.25)
    result.refresh(0, force=True)
    figure.canvas.draw()
    return result


def click(plot, xy):
    x, y = plot.map_ax.transData.transform(xy)
    event = MouseEvent("button_press_event", plot.figure.canvas, x, y, button=1)
    plot.figure.canvas.callbacks.process("button_press_event", event)


def test_click_shows_plotted_xy_and_tracks_position_refinements(plot):
    click(plot, (-1.25, 2.5))
    assert plot.tag_coordinates.get_visible()
    assert plot.tag_coordinates.get_text() == "Tag 5\nX (right): -1.25 m\nY (forward): +2.50 m"
    plot.map.tags[5] = (2.4, -1.3)
    plot.refresh(.2)
    assert plot.tag_coordinates.xy == (-1.3, 2.4)
    assert "X (right): -1.30 m\nY (forward): +2.40 m" in plot.tag_coordinates.get_text()


def test_label_can_be_clicked_and_empty_space_dismisses_coordinates(plot):
    label = plot.map_artists[5][1]
    box = label.get_window_extent(plot.figure.canvas.get_renderer())
    event = MouseEvent("button_press_event", plot.figure.canvas,
                       (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2, button=1)
    plot.figure.canvas.callbacks.process("button_press_event", event)
    assert plot.selected_tag == 5
    click(plot, (2, -2))
    assert not plot.tag_coordinates.get_visible()


def test_reset_clears_selection_and_pan_does_not_select_tags(plot):
    plot.figure.canvas.toolbar = SimpleNamespace(mode="pan/zoom")
    click(plot, (-1.25, 2.5))
    assert plot.selected_tag is None
    plot.figure.canvas.toolbar = None
    click(plot, (-1.25, 2.5))
    assert plot.selected_tag == 5
    plot.map.reset()
    plot.refresh(.2)
    assert plot.selected_tag is None
    assert not plot.tag_coordinates.get_visible()


def test_trajectory_key_stays_above_plot_and_includes_csv_preview(plot):
    plot.map_ax.plot([], [], "--", label="CSV preview")
    plot.update_map_legend()
    plot.figure.canvas.draw()
    key = plot.map_ax.get_legend()
    box = key.get_window_extent(plot.figure.canvas.get_renderer())
    assert box.y0 > plot.map_ax.bbox.y1
    assert box.y1 <= plot.figure.bbox.y1
    assert "CSV preview" in [label.get_text() for label in key.get_texts()]
