var Drawer = function() {
    this.canvas = null;
    this.ctx = null;
    this.w = 0;
    this.h = 0;
    this.stroke_style = "";
    this.stroke_width = 2;
    this.prevX = 0;
    this.prevY = 0;
    this.currX = 0;
    this.currY = 0;
}

Drawer.prototype.init = function (canvas, initial_x, initial_y) {
    this.canvas = canvas;
    this.ctx = this.canvas.getContext("2d");
    this.w = this.canvas.width;
    this.h = this.canvas.height;
    this.stroke_style = "black";
    this.stroke_width = 2;
    this.prevX = initial_x;
    this.currX = initial_x;
    this.prevY = initial_y;
    this.currY = initial_y;
}

Drawer.prototype.draw = function () {
    if (this.prevX != this.currX || this.prevY != this.currY)    {
        this.ctx.beginPath();
        this.ctx.moveTo(this.prevX, this.prevY);
        this.ctx.lineTo(this.currX, this.currY);
        this.ctx.strokeStyle = this.stroke_style;
        this.ctx.lineWidth = this.stroke_width;
        this.ctx.stroke();
        this.ctx.closePath();
    }
}

Drawer.prototype.erase = function () {
    this.ctx.clearRect(0, 0, w, h);
}

Drawer.prototype.gotoxy = function(x, y) {
    this.prevX = this.currX;
    this.prevY = this.currY;
    this.currX = x;
    this.currY = y;
}
