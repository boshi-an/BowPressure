import qrcode

web_url = "https://boshi-an.github.io/BowPressure/"
form_url = "https://docs.google.com/forms/d/e/1FAIpQLScBvg3L9aCwDaN2HkErQFBNhkxiTNuEm5pRcO5IgzcGij9dhA/viewform"

img = qrcode.make(web_url)
img.save("qr_code_web.png")

img = qrcode.make(form_url)
img.save("qr_code_form.png")
