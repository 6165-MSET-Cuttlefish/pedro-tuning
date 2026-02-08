package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class EnhancedTelemetry implements Telemetry {
    private final Telemetry dsTelemetry;
    private final Telemetry dashTelemetry;

    public EnhancedTelemetry(Telemetry dsTelemetry, Telemetry dashTelemetry) {
        this.dsTelemetry = dsTelemetry;
        this.dashTelemetry = dashTelemetry;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        List<Item> items = new ArrayList<>();
        items.add(dsTelemetry.addData(caption, format, args));
        items.add(dashTelemetry.addData(caption, format, args));
        return new EnhancedItem(items);
    }

    @Override
    public Item addData(String caption, Object value) {
        List<Item> items = new ArrayList<>();
        items.add(dsTelemetry.addData(caption, value));
        items.add(dashTelemetry.addData(caption, value));
        return new EnhancedItem(items);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        List<Item> items = new ArrayList<>();
        items.add(dsTelemetry.addData(caption, valueProducer));
        items.add(dashTelemetry.addData(caption, valueProducer));
        return new EnhancedItem(items);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        List<Item> items = new ArrayList<>();
        items.add(dsTelemetry.addData(caption, format, valueProducer));
        items.add(dashTelemetry.addData(caption, format, valueProducer));
        return new EnhancedItem(items);
    }

    @Override
    public boolean removeItem(Item item) {
        boolean r1 = dsTelemetry.removeItem(item);
        boolean r2 = dashTelemetry.removeItem(item);
        return r1 || r2;
    }

    @Override
    public void clear() {
        dsTelemetry.clear();
        dashTelemetry.clear();
    }

    @Override
    public void clearAll() {
        dsTelemetry.clearAll();
        dashTelemetry.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        dsTelemetry.addAction(action);
        dashTelemetry.addAction(action);
        return action;
    }

    @Override
    public boolean removeAction(Object token) {
        boolean r1 = dsTelemetry.removeAction(token);
        boolean r2 = dashTelemetry.removeAction(token);
        return r1 || r2;
    }

    @Override
    public void speak(String text) {
        dsTelemetry.speak(text);
        dashTelemetry.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        dsTelemetry.speak(text, languageCode, countryCode);
        dashTelemetry.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        boolean r1 = dsTelemetry.update();
        boolean r2 = dashTelemetry.update();
        return r1 && r2;
    }

    @Override
    public Line addLine() {
        List<Line> lines = new ArrayList<>();
        lines.add(dsTelemetry.addLine());
        lines.add(dashTelemetry.addLine());
        return new EnhancedLine(lines);
    }

    @Override
    public Line addLine(String lineCaption) {
        List<Line> lines = new ArrayList<>();
        lines.add(dsTelemetry.addLine(lineCaption));
        lines.add(dashTelemetry.addLine(lineCaption));
        return new EnhancedLine(lines);
    }

    @Override
    public boolean removeLine(Line line) {
        boolean r1 = dsTelemetry.removeLine(line);
        boolean r2 = dashTelemetry.removeLine(line);
        return r1 || r2;
    }

    @Override
    public boolean isAutoClear() {
        return dsTelemetry.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        dsTelemetry.setAutoClear(autoClear);
        dashTelemetry.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return dsTelemetry.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        dsTelemetry.setMsTransmissionInterval(msTransmissionInterval);
        dashTelemetry.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return dsTelemetry.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        dsTelemetry.setItemSeparator(itemSeparator);
        dashTelemetry.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return dsTelemetry.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        dsTelemetry.setCaptionValueSeparator(captionValueSeparator);
        dashTelemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        dsTelemetry.setDisplayFormat(displayFormat);
        dashTelemetry.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return new CombinedLog(dsTelemetry.log(), dashTelemetry.log());
    }

    public EnhancedTelemetry addDSData(String caption, Object value) {
        dsTelemetry.addData(caption, value);
        return this;
    }

    public EnhancedTelemetry addDSData(String caption, String format, Object... args) {
        dsTelemetry.addData(caption, format, args);
        return this;
    }

    public EnhancedTelemetry addDashboardData(String caption, Object value) {
        dashTelemetry.addData(caption, value);
        return this;
    }

    public EnhancedTelemetry addDashboardData(String caption, String format, Object... args) {
        dashTelemetry.addData(caption, format, args);
        return this;
    }

    private static class EnhancedItem implements Item {
        private final List<Item> items;

        EnhancedItem(List<Item> items) {
            this.items = items;
        }

        @Override
        public String getCaption() {
            return items.get(0).getCaption();
        }

        @Override
        public Item setCaption(String caption) {
            for (Item item : items) {
                item.setCaption(caption);
            }
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            for (Item item : items) {
                item.setValue(format, args);
            }
            return this;
        }

        @Override
        public Item setValue(Object value) {
            for (Item item : items) {
                item.setValue(value);
            }
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            for (Item item : items) {
                item.setValue(valueProducer);
            }
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            for (Item item : items) {
                item.setValue(format, valueProducer);
            }
            return this;
        }

        @Override
        public Item setRetained(@Nullable Boolean retained) {
            for (Item item : items) {
                item.setRetained(retained);
            }
            return this;
        }

        @Override
        public boolean isRetained() {
            return items.get(0).isRetained();
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            for (Item item : items) {
                item.addData(caption, format, args);
            }
            return this;
        }

        @Override
        public Item addData(String caption, Object value) {
            for (Item item : items) {
                item.addData(caption, value);
            }
            return this;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            for (Item item : items) {
                item.addData(caption, valueProducer);
            }
            return this;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            for (Item item : items) {
                item.addData(caption, format, valueProducer);
            }
            return this;
        }
    }

    private static class EnhancedLine implements Line {
        private final List<Line> lines;

        EnhancedLine(List<Line> lines) {
            this.lines = lines;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            List<Item> items = new ArrayList<>();
            for (Line line : lines) {
                items.add(line.addData(caption, format, args));
            }
            return new EnhancedItem(items);
        }

        @Override
        public Item addData(String caption, Object value) {
            List<Item> items = new ArrayList<>();
            for (Line line : lines) {
                items.add(line.addData(caption, value));
            }
            return new EnhancedItem(items);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            List<Item> items = new ArrayList<>();
            for (Line line : lines) {
                items.add(line.addData(caption, valueProducer));
            }
            return new EnhancedItem(items);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            List<Item> items = new ArrayList<>();
            for (Line line : lines) {
                items.add(line.addData(caption, format, valueProducer));
            }
            return new EnhancedItem(items);
        }
    }

    private static class CombinedLog implements Log {
        private final Log dsLog;
        private final Log dashLog;

        CombinedLog(Log dsLog, Log dashLog) {
            this.dsLog = dsLog;
            this.dashLog = dashLog;
        }

        @Override
        public int getCapacity() {
            return dsLog.getCapacity();
        }

        @Override
        public void setCapacity(int capacity) {
            dsLog.setCapacity(capacity);
            dashLog.setCapacity(capacity);
        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return dsLog.getDisplayOrder();
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {
            dsLog.setDisplayOrder(displayOrder);
            dashLog.setDisplayOrder(displayOrder);
        }

        @Override
        public void add(String message) {
            dsLog.add(message);
            dashLog.add(message);
        }

        @Override
        public void add(String format, Object... args) {
            dsLog.add(format, args);
            dashLog.add(format, args);
        }

        @Override
        public void clear() {
            dsLog.clear();
            dashLog.clear();
        }
    }
}
