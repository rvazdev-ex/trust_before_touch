import qrcode

# URL you want to encode
url = "https://github.com/rvazdev-ex/trust_before_touch"

# Create QR code object
qr = qrcode.QRCode(
    version=1,  # controls size (1–40)
    error_correction=qrcode.constants.ERROR_CORRECT_Q,
    box_size=10,
    border=4,
)

qr.add_data(url)
qr.make(fit=True)

# Generate image
img = qr.make_image(fill_color="black", back_color="white")

# Save to file
img.save("trust_before_touch_qr.png")

print("QR code saved as trust_before_touch_qr.png")
