

import numpy as np
from PIL import Image, ImageFont, ImageDraw, ImageEnhance

# plots the engagement value on the images, with a window of history
class TimeSerieInImage(object):

    def __init__(self):

        self.data_serie = np.empty((0))

        self.line_width = 3
        self.plot_max_height_ratio = 0.1
        self.plot_offset_ratio = 0.89
        
        # num_points_to_plot before and num_points_to_plot after the current frame
        self.num_points_to_plot = 50
        self.pre_color = (0, 0, 255, 255)
        self.post_color = (0, 0, 100, 255)

        self.font = ImageFont.load_default()


    def step(self, frame, value):
        img = Image.fromarray(frame)

        self.data_serie = np.concatenate((self.data_serie, [value]))
        if self.data_serie.size > self.num_points_to_plot + 1:
            self.data_serie = np.delete(self.data_serie, 0, 0)

        (width, height) = img.size
        plot_max_height = self.plot_max_height_ratio * height
        plot_offset = self.plot_offset_ratio * height

        step = float(width) / (self.num_points_to_plot * 2)
        x = [step*i for i in range(self.num_points_to_plot * 2 + 1)]

        # draw the plot grid
        grid_draw = ImageDraw.Draw(img)
        grid_draw.line([(0, plot_offset), (width, plot_offset)], fill=(255, 255, 255), width=2)
        grid_draw.line([(0, plot_offset + plot_max_height), (width, plot_offset + plot_max_height)], fill=(255, 255, 255), width=2)
        grid_draw.line([(width/2., plot_offset - 10), (width/2., plot_offset + plot_max_height + 1)], fill=(200, 200, 200), width=2)
        grid_draw.text((0, plot_offset - 11), "1.0", font=self.font, fill=(255, 255, 255))
        grid_draw.text((0, plot_offset + plot_max_height - 11), "0.0", font=self.font, fill=(255, 255, 255))

        # plot the series
        y_pre  = self.data_serie[0:min(self.num_points_to_plot, self.data_serie.size)]
        y_pre = 1. - y_pre
        y_pre *= float(plot_max_height)
        y_pre += plot_offset
        x_pre = x[self.num_points_to_plot - y_pre.size + 1:self.num_points_to_plot + 1]

        y_post = np.array([value for _ in range(self.num_points_to_plot)])
        y_post = 1. - y_post
        y_post *= float(plot_max_height)
        y_post += plot_offset
        x_post = x[self.num_points_to_plot:self.num_points_to_plot + y_post.size + 1]

        xy_pre = [el for el in zip(x_pre, y_pre)]
        xy_post = [el for el in zip(x_post, y_post)]

        plot_draw = ImageDraw.Draw(img)
        plot_draw.line(xy_pre, fill=self.pre_color, width=self.line_width, joint="curve")
        plot_draw.line(xy_post, fill=self.post_color, width=self.line_width, joint="curve")
        plot_draw.text((width/2., plot_offset - 20  ), "%.2f" % value, font=self.font, fill=self.pre_color)

        return np.array(img)
