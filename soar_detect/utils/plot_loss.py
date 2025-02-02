import matplotlib.pyplot as plt
import numpy as np



epoch = np.arange(1,26)
loss = np.array(
    [
        0.0192,
        0.0168,
        0.0154,
        0.0148,
        0.0145,
        0.0141,
        0.0135,
        0.0130,
        0.0126,
        0.0123,
        0.0120,
        0.0118,
        0.0116,
        0.0114,
        0.0112,
        0.0110,
        0.0108,
        0.0106,
        0.0104,
        0.0102,
        0.0099,
        0.0096,
        0.0094,
        0.0091,
        0.0089
    ]
)

fig,ax = plt.subplots(figsize=(10, 5))
ax.plot(epoch, loss, color='orange')

ax.grid(True)

ax.set_xlabel('Epoch')
ax.set_ylabel('Loss')
ax.set_title('Training Loss')
ax.set_xlim([0, 25])
plt.tight_layout()
plt.savefig('utils/loss_plot.png', dpi=300)