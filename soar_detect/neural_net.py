import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
import os
from PIL import Image
import numpy as np


class DoubleConv(nn.Module):
    """(Convolution => BatchNorm => ReLU) * 2"""
    def __init__(self, in_channels, out_channels):
        super(DoubleConv, self).__init__()
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),

            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.double_conv(x)

class Down(nn.Module):
    """Downscaling with MaxPool followed by DoubleConv"""
    def __init__(self, in_channels, out_channels):
        super(Down, self).__init__()
        self.down_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_channels, out_channels)
        )

    def forward(self, x):
        return self.down_conv(x)

class Up(nn.Module):
    """Upscaling followed by DoubleConv"""
    def __init__(self, in_channels, out_channels):
        super(Up, self).__init__()
        self.up_sample = nn.ConvTranspose2d(in_channels // 2, in_channels // 2, kernel_size=2, stride=2)
        self.conv = DoubleConv(in_channels, out_channels)

    def forward(self, x1, x2):
        x1 = self.up_sample(x1)
        # Handle padding if needed
        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]
        x1 = nn.functional.pad(x1, [diffX // 2, diffX - diffX //2,
                                    diffY // 2, diffY - diffY //2])
        x = torch.cat([x2, x1], dim=1)
        return self.conv(x)

class OutConv(nn.Module):
    """Final Convolution layer to get the desired output channels"""
    def __init__(self, in_channels, out_channels):
        super(OutConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1)

    def forward(self, x):
        return self.conv(x)

class UNet(nn.Module):
    def __init__(self, n_channels=1, n_classes=1):
        super(UNet, self).__init__()
        self.n_channels = n_channels
        self.n_classes = n_classes

        # UNet encoder layers
        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        self.down4 = Down(512, 512)

        # Fully connected layers for wind vector
        self.wind_fc = nn.Sequential(
            nn.Linear(2, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 64),
            nn.ReLU(inplace=True),
        )
        self.adjust_conv = nn.Conv2d(576, 512, kernel_size=1)

        # Adjusted decoder layers to accommodate concatenated features
        self.up1 = Up(512 + 512, 256)
        self.up2 = Up(256 + 256, 128)
        self.up3 = Up(128 + 128, 64)
        self.up4 = Up(64 + 64, 64)

        self.outc = OutConv(64, n_classes)

    def forward(self, x, wind_vector):
        # x: depth image, shape (B, 1, H, W)
        # wind_vector: shape (B, 2)

        # Encoder path
        x1 = self.inc(x)       # (B, 64, H, W)
        x2 = self.down1(x1)    # (B, 128, H/2, W/2)
        x3 = self.down2(x2)    # (B, 256, H/4, W/4)
        x4 = self.down3(x3)    # (B, 512, H/8, W/8)
        x5 = self.down4(x4)    # (B, 512, H/16, W/16)

        # Process wind vector
        wind_features = self.wind_fc(wind_vector)  # (B, 64)
        wind_features = wind_features.unsqueeze(2).unsqueeze(3)  # (B, 64, 1, 1)
        wind_features = wind_features.expand(-1, -1, x5.size(2), x5.size(3))  # (B, 64, H/16, W/16)

        # Concatenate wind features with x5
        x5 = torch.cat([x5, wind_features], dim=1)  # (B, 512 + 64, H/16, W/16)
        x5 = self.adjust_conv(x5)  

        # Decoder path
        x = self.up1(x5, x4)   # x5: (B, 576, H/16, W/16), x4: (B, 512, H/8, W/8)
        x = self.up2(x, x3)    # x: (B, 256, H/8, W/8), x3: (B, 256, H/4, W/4)
        x = self.up3(x, x2)    # x: (B, 128, H/4, W/4), x2: (B, 128, H/2, W/2)
        x = self.up4(x, x1)    # x: (B, 64, H/2, W/2), x1: (B, 64, H, W)
        logits = self.outc(x)  # (B, n_classes, H, W)
        return logits
