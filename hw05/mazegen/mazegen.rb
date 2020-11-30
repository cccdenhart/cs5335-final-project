#!/usr/bin/env ruby

require 'erb'

class MazeGen
  def initialize()
    @wall_ii = 0
  end

  def next_ii
    i0 = @wall_ii
    @wall_ii += 1
    i0
  end

  def wall(xx, yy, rot=false)
    aa = rot ? 0 : (Math::PI/2)
    %Q{
    <include>
      <uri>model://brick_box_3x1x3</uri>
      <name>wall#{(next_ii)}</name>
      <pose>#{xx} #{yy} -2.0 0 0 #{aa}</pose>
    </include>
    }
  end

  def roll
    1 + rand(4)
  end

  def gen
    tpl = ERB.new(File.read("template.xml.erb"), trim_mode: "-<>")
    walls = []
    (-6).upto(6) do |ii|
      xx = 2 + ii * 4
      skips = [-roll(), roll()]
      (-6).upto(6) do |jj|
        yy = jj * 3
        if ii.abs == 6 then
          walls.push(wall(xx, yy))
        else
          if skips.include?(jj) then
            dy = (jj.even?) ? 2 : -2
            if skips[ii % 2] == jj && ii != 5 then
              walls.push(wall(xx + 2, yy + dy, true))
            end
          else
            walls.push(wall(xx, yy))
          end
        end
      end
      if ii != 6 then
        walls.push(wall(xx + 2, -19, true))
        walls.push(wall(xx + 2, 19, true))
      end
    end

    puts tpl.run(binding)
  end
end

mg = MazeGen.new
mg.gen
